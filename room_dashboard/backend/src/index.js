import mqtt from "mqtt";

export default {
  async fetch(request, env) {
    if (request.headers.get("Upgrade") !== "websocket") {
      return new Response("WebSocket only", {status: 400});
    }
    const [client, server] = new WebSocketPair();
    server.accept();
	const mqttClient = mqtt.connect({
	protocol: "wss",
	hostname: env.MQTT_HOST,
	port: 8884,
	path: "/mqtt",
	username: env.MQTT_USERNAME,
	password: env.MQTT_PASSWORD,
	});
    mqttClient.on("connect", () => {
      mqttClient.subscribe("room/telemetry");
    });
    mqttClient.on("message", (_topic, payload) => {
      server.send(payload.toString());
    });
    server.addEventListener("close", () => {
      mqttClient.end();
    });

    return new Response(null, {
      status: 101,
      webSocket: client,
    });
  },
};
