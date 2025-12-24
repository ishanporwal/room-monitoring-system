#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"

#include "secrets.h"

#define WIFI_CONNECTED_BIT BIT0

#define UART_RX_PIN GPIO_NUM_16
#define UART_TX_PIN GPIO_NUM_17
#define UART_PORT UART_NUM_2
#define UART_BAUD 115200
#define UART_START_BYTE 0xAA

static const char *TAG = "GATEWAY";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static EventGroupHandle_t wifi_event_group;
static bool mqtt_connected = false;

/* -------------------- Data Structures -------------------- */

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

/* -------------------- MQTT Publish -------------------- */

static void publish_room_state(const room_packet_t *pkt)
{
    if (!mqtt_client || !mqtt_connected) {
        ESP_LOGW("MQTT", "Publish skipped (MQTT not ready)");
        return;
    }

    char msg[256];

    snprintf(msg, sizeof(msg),
        "{"
        "\"occupied\":%d,"
        "\"light_on\":%d,"
        "\"temperature_f\":%.1f,"
        "\"humidity\":%.1f,"
        "\"need_heating\":%d,"
        "\"need_cooling\":%d,"
        "\"need_humidifier\":%d"
        "}",
        pkt->occupied,
        pkt->light_on,
        pkt->temperature_f,
        pkt->humidity,
        pkt->need_heating,
        pkt->need_cooling,
        pkt->need_humidifier
    );

    esp_mqtt_client_publish(
        mqtt_client,
        "room/telemetry",
        msg,
        0,
        1,
        0
    );
}

/* -------------------- Wi-Fi -------------------- */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW("WIFI", "Disconnected, retrying");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI("WIFI", "Got IP address");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void init_wifi(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI("WIFI", "Wi-Fi init complete");
}

/* -------------------- MQTT -------------------- */

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI("MQTT", "Connected to broker");
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW("MQTT", "Disconnected");
            break;

        default:
            break;
    }
}

static void init_mqtt(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication.password = MQTT_PASSWORD,
        }
    };
    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI("MQTT", "MQTT client started");
}

/* -------------------- UART -------------------- */

static void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(
        UART_PORT,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
}

void uart_rx_task(void *arg) {
    uint8_t byte;
    uart_frame_t frame;
    uint8_t *p = (uint8_t *)&frame;
    int idx = 0;

    while (1) {
        uart_read_bytes(UART_PORT, &byte, 1, portMAX_DELAY);
        if (idx == 0 && byte != UART_START_BYTE) {
            continue;
        }
        p[idx++] = byte;
        if (idx == sizeof(uart_frame_t)) {
            if (frame.start == UART_START_BYTE &&
                frame.length == sizeof(room_packet_t)) {
                room_packet_t *pkt = &frame.payload;
                ESP_LOGI(TAG,
                    "Occ=%d Light=%d Temp=%.1fF Hum=%.1f%% Heat=%d Cool=%d Hum=%d",
                    pkt->occupied,
                    pkt->light_on,
                    pkt->temperature_f,
                    pkt->humidity,
                    pkt->need_heating,
                    pkt->need_cooling,
                    pkt->need_humidifier
                );
                publish_room_state(pkt);
            }
            idx = 0;
        }
    }
}

/* -------------------- Tasks -------------------- */

void mqtt_task(void *arg) {
    ESP_LOGI("MQTT", "Waiting for Wi-Fi...");
    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        false,
        true,
        portMAX_DELAY
    );
    ESP_LOGI("MQTT", "Wi-Fi ready, starting MQTT");
    init_mqtt();
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting gateway...");

    init_wifi();
    init_uart();

    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);
}
