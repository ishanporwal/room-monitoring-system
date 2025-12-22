#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "dht.h"
#include "driver/uart.h"

// pin defs
#define PIR_GPIO GPIO_NUM_27
#define LDR_ADC_CHANNEL ADC1_CHANNEL_6
#define DHT_GPIO GPIO_NUM_26
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16
#define UART_PORT UART_NUM_2
#define UART_START_BYTE 0xAA
#define UART_BAUD 115200

static const char *TAG = "ROOM_MONITOR";

// events
typedef enum {
    EVENT_MOTION,
    EVENT_LIGHT,
    EVENT_TEMP,
    EVENT_HUMIDITY
} event_type_t;

typedef struct{
    event_type_t type;
    int value;
    int64_t timestamp_ms;
} sensor_event_t;

// data
typedef struct{
    bool occupied;
    bool light_on;
    float temperature_f;
    float humidity;
    bool need_heating;
    bool need_cooling;
    bool need_humidifier;

    int64_t last_update_ms;
} room_state_t;

static room_state_t g_room_state;
static SemaphoreHandle_t room_state_mutex;

typedef struct __attribute__((packed)) {
    uint8_t occupied;
    uint8_t light_on;
    float temperature_f;
    float humidity;
    uint8_t need_heating;
    uint8_t need_cooling;
    uint8_t need_humidifier;
} room_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t start;
    uint8_t length;
    room_packet_t payload;
} uart_frame_t;

//  queue
static QueueHandle_t event_queue;

// PIR ISR
static void IRAM_ATTR pir_isr_handler(void *arg){
    BaseType_t higher_woken = pdFALSE;
    sensor_event_t evt = {
        .type = EVENT_MOTION,
        .value = 1,
        .timestamp_ms = esp_timer_get_time() / 1000
    };
    xQueueSendFromISR(event_queue, &evt, &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
}

// init funcs
static void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void init_pir(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIR_GPIO, pir_isr_handler, NULL);
}

static void init_ldr(void){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

// Tasks
void light_task(void *arg) {
    while (1) {
        sensor_event_t evt = {
            .type = EVENT_LIGHT,
            .value = adc1_get_raw(LDR_ADC_CHANNEL),
            .timestamp_ms = esp_timer_get_time() / 1000
        };
        xQueueSend(event_queue, &evt, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void temp_task(void *arg) {
    while (1) {
        int16_t temp_x10 = 0;
        int16_t hum_x10 = 0;
        esp_err_t res = dht_read_data(
            DHT_TYPE_DHT11, DHT_GPIO, &hum_x10, &temp_x10
        );
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (res == ESP_OK) {
            sensor_event_t evt_t = {
                .type = EVENT_TEMP,
                .value = temp_x10,
                .timestamp_ms = now_ms
            };

            sensor_event_t evt_h = {
                .type = EVENT_HUMIDITY,
                .value = hum_x10,
                .timestamp_ms = now_ms
            };

            xQueueSend(event_queue, &evt_t, 0);
            xQueueSend(event_queue, &evt_h, 0);
        }
        else {
            ESP_LOGW(TAG, "DHT11 read failed: %s", esp_err_to_name(res));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void logic_task(void *arg) {
    sensor_event_t evt;

    bool occupied = false;
    int64_t last_motion_time = 0;

    bool light_on = false;
    int last_light = -1;
    const int LIGHT_CHANGE = 400;
    const int LIGHT_THRES = 1500;

    bool need_heating = false;
    bool need_cooling = false;
    bool need_hum = false;
    const float HEAT_THRES = 81.0f;
    const float COOL_THRES = 82.0f;
    float cur_temp_F = 0.0f;
    float cur_hum = 0.0f;

    while (1) {
        if (xQueueReceive(event_queue, &evt, portMAX_DELAY)) {
            switch (evt.type) {
                case EVENT_MOTION:
                    if (!occupied) {
                        occupied = true;
                        ESP_LOGI("LOGIC", "Room is now occupied");
                    }
                    last_motion_time = evt.timestamp_ms;
                    break;

                case EVENT_LIGHT: {
                    ESP_LOGI("LOGIC", "Light is %d", evt.value);
                    if (last_light == -1) {
                        last_light = evt.value;
                        light_on = (evt.value > LIGHT_THRES);

                        ESP_LOGI("LOGIC",
                            "Initial light state: %s (ADC=%d)",
                            light_on ? "ON" : "OFF",
                            evt.value
                        );
                        break;
                    }

                    int diff = evt.value - last_light;

                    if (!light_on && evt.value > LIGHT_THRES && diff > LIGHT_CHANGE) {
                        light_on = true;
                        ESP_LOGI("LOGIC", "Light turned ON (ADC=%d, Δ=%d)", evt.value, diff);
                    }
                    else if (light_on && evt.value <= LIGHT_THRES && diff < -LIGHT_CHANGE) {
                        light_on = false;
                        ESP_LOGI("LOGIC", "Light turned OFF (ADC=%d, Δ=%d)", evt.value, diff);
                    }
                    last_light = evt.value;
                    break;
                }

                case EVENT_TEMP: {
                    float tempC = evt.value / 10.0f;
                    float tempF = tempC * 9.0f / 5.0f + 32.0f;
                    cur_temp_F = tempF;
                    if (tempF < HEAT_THRES && !need_heating) {
                        need_heating = true;
                        need_cooling = false;
                        ESP_LOGI("LOGIC", "Heating needed (%.1f°F)", tempF);
                    }
                    else if (tempF > COOL_THRES && !need_cooling) {
                        need_cooling = true;
                        need_heating = false;
                        ESP_LOGI("LOGIC", "Cooling needed (%.1f°F)", tempF);
                    }
                    else if (tempF >= HEAT_THRES && tempF <= COOL_THRES) {
                        if (need_heating || need_cooling) {
                            ESP_LOGI("LOGIC", "Temp stable (%.1f°F)", tempF);
                        }
                        need_heating = false;
                        need_cooling = false;
                    }
                    break;
                }

                case EVENT_HUMIDITY: {
                    float hum = evt.value / 10.0f;
                    cur_hum = hum;
                    if (hum < 30.0f && !need_hum) {
                        need_hum = true;
                        ESP_LOGI("LOGIC", "Humidifier needed (%.1f%%)", hum);
                    }
                    else if (hum > 55.0f && need_hum) {
                        need_hum = false;
                        ESP_LOGI("LOGIC", "Humidifier not needed (%.1f%%)", hum);
                    }
                    break;
                }

                default:
                    break;
            }
            if (occupied && (evt.timestamp_ms - last_motion_time > 120000)) {
                occupied = false;
                ESP_LOGI("LOGIC", "Room is EMPTY (timeout)");
            }

            xSemaphoreTake(room_state_mutex, portMAX_DELAY);
            g_room_state.occupied = occupied;
            g_room_state.light_on = light_on;
            g_room_state.temperature_f = cur_temp_F;
            g_room_state.humidity = cur_hum;
            g_room_state.need_heating = need_heating;
            g_room_state.need_cooling = need_cooling;
            g_room_state.need_humidifier = need_hum;
            g_room_state.last_update_ms = evt.timestamp_ms;
            xSemaphoreGive(room_state_mutex);
        }
    }
}

void uart_tx_task(void *arg) {
    uart_frame_t frame;

    frame.start = UART_START_BYTE;
    frame.length = sizeof(room_packet_t);

    while (1) {
        xSemaphoreTake(room_state_mutex, portMAX_DELAY);
        frame.payload.occupied        = g_room_state.occupied;
        frame.payload.light_on        = g_room_state.light_on;
        frame.payload.temperature_f   = g_room_state.temperature_f;
        frame.payload.humidity        = g_room_state.humidity;
        frame.payload.need_heating    = g_room_state.need_heating;
        frame.payload.need_cooling    = g_room_state.need_cooling;
        frame.payload.need_humidifier = g_room_state.need_humidifier;
        xSemaphoreGive(room_state_mutex);

        uart_write_bytes(UART_PORT, &frame, sizeof(frame));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// entry -> init hardware and create RTOS tasks
void app_main(void) {
    ESP_LOGI(TAG, "Starting room monitoring...");
    event_queue = xQueueCreate(16, sizeof(sensor_event_t));
    room_state_mutex = xSemaphoreCreateMutex();

    init_uart();
    init_pir();
    init_ldr();
    
    xTaskCreate(light_task, "light_task", 2048, NULL, 4, NULL);
    xTaskCreate(temp_task, "temp_task", 4096, NULL, 4, NULL);
    xTaskCreate(logic_task, "logic task", 4096, NULL, 3, NULL);
    xTaskCreate(uart_tx_task, "uart_tx", 2048, NULL, 2, NULL);
    ESP_LOGI(TAG, "Monitoring");
}
