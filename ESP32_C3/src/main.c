#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/data/json.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(mqtt_client, LOG_LEVEL_DBG);

#define WIFI_SSID "Jake_iphone"
#define WIFI_PSK "bbbooooo"

#define MQTT_BROKER_ADDR "172.20.10.11"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "esp32c3_client"
#define MQTT_TOPIC_PUB "esp32c3/sensors"
#define MQTT_TOPIC_SUB "esp32c3/command"

static struct net_mgmt_event_callback wifi_cb;

static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];
static struct mqtt_client client_ctx;
static struct sockaddr_storage broker;
static struct zsock_pollfd fds[1];
static int nfds;
static bool connected = false;

static bool wifi_connected_flag = false;
static struct net_if *sta_iface = NULL;

#define NET_EVENT_WIFI_MASK \
	(NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		LOG_INF("Connected to %s", WIFI_SSID);
		wifi_connected_flag = true;
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		LOG_INF("Disconnected from %s", WIFI_SSID);
		wifi_connected_flag = false;
		break;
	default:
		break;
	}
}

static int connect_to_wifi(void)
{
	sta_iface = net_if_get_wifi_sta();
	if (!sta_iface) {
		LOG_ERR("STA: interface not initialized");
		return -EIO;
	}

	struct wifi_connect_req_params params = {
		.ssid = (const uint8_t *)WIFI_SSID,
		.ssid_length = strlen(WIFI_SSID),
		.psk = (const uint8_t *)WIFI_PSK,
		.psk_length = strlen(WIFI_PSK),
		.security = WIFI_SECURITY_TYPE_PSK,
		.channel = WIFI_CHANNEL_ANY,
		.band = WIFI_FREQ_BAND_2_4_GHZ,
	};

	LOG_INF("Connecting to SSID: %s", WIFI_SSID);
	int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &params, sizeof(params));
	if (ret) {
		LOG_ERR("Unable to connect to WiFi");
	}
	return ret;
}

static void broker_init(void)
{
	struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;
	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(MQTT_BROKER_PORT);
	zsock_inet_pton(AF_INET, MQTT_BROKER_ADDR, &broker4->sin_addr);
}

static void client_init(struct mqtt_client *client)
{
	mqtt_client_init(client);
	broker_init();

	client->broker = &broker;
	client->evt_cb = NULL;
	client->client_id.utf8 = (uint8_t *)MQTT_CLIENT_ID;
	client->client_id.size = strlen(MQTT_CLIENT_ID);
	client->password = NULL;
	client->user_name = NULL;
	client->protocol_version = MQTT_VERSION_3_1_1;

	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

static int publish_message(struct mqtt_client *client, const char *topic, const char *message)
{
	struct mqtt_publish_param param = {
		.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE,
		.message.topic.topic.utf8 = (uint8_t *)topic,
		.message.topic.topic.size = strlen(topic),
		.message.payload.data = message,
		.message.payload.len = strlen(message),
		.message_id = sys_rand32_get(),
		.dup_flag = 0U,
		.retain_flag = 0U,
	};
	return mqtt_publish(client, &param);
}

static void poll_mqtt_client(void)
{
	if (zsock_poll(fds, nfds, 100) < 0) return;
	if ((fds[0].revents & ZSOCK_POLLIN)) mqtt_input(&client_ctx);
	if ((fds[0].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP))) connected = false;
}

struct SensorSet {
	float temp;
	float humidity;
	float co2;
};

struct SensorPayload {
	struct SensorSet things[4];
	int moisture1;
	int moisture2;
};

static int create_sensor_json(char *out, size_t size, struct SensorPayload *payload)
{
	JSON_OBJ_DESCR_DEFINE_ARRAY(things_descr, 4, struct SensorSet);
	JSON_OBJ_DESCR_DEFINE(struct SensorPayload, sensor_descr,
		JSON_OBJ_DESCR_PRIM_NAMED(struct SensorPayload, "thingy52", things, JSON_TOK_LIST, struct SensorSet[4]),
		JSON_OBJ_DESCR_PRIM(struct SensorPayload, moisture1, JSON_TOK_NUMBER),
		JSON_OBJ_DESCR_PRIM(struct SensorPayload, moisture2, JSON_TOK_NUMBER)
	);

	return json_obj_encode_buf(sensor_descr, ARRAY_SIZE(sensor_descr), payload, out, size);
}

static void mqtt_thread(void)
{
	net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler, NET_EVENT_WIFI_MASK);
	net_mgmt_add_event_callback(&wifi_cb);
	connect_to_wifi();

	while (!wifi_connected_flag) k_msleep(100);
	LOG_INF("WiFi connected");

	client_init(&client_ctx);
	if (mqtt_connect(&client_ctx) != 0) {
		LOG_ERR("MQTT connection failed");
		return;
	}

	fds[0].fd = client_ctx.transport.tcp.sock;
	fds[0].events = ZSOCK_POLLIN;
	nfds = 1;

	while (!connected) {
		poll_mqtt_client();
		connected = true;
		k_msleep(100);
	}

	char json_buf[512];
	uint64_t last_pub = 0;

	while (1) {
		poll_mqtt_client();
		mqtt_live(&client_ctx);

		// Simulate receiving sensor data from BLE (to be replaced with real data)
		struct SensorPayload payload = {
			.things = {
				{22.5, 55.0, 500.0}, {23.0, 54.0, 510.0},
				{22.8, 53.5, 495.0}, {23.2, 52.0, 505.0}
			},
			.moisture1 = 700,
			.moisture2 = 685
		};

		uint64_t now = k_uptime_get();
		if (now - last_pub >= 100) {
			if (create_sensor_json(json_buf, sizeof(json_buf), &payload) == 0) {
				int ret = publish_message(&client_ctx, MQTT_TOPIC_PUB, json_buf);
				if (ret != 0) {
					LOG_ERR("Failed to publish sensor data: %d", ret);
				} else {
					LOG_INF("Published sensor data");
				}
			} else {
				LOG_ERR("Failed to encode JSON");
			}
			last_pub = now;
		}

		k_msleep(10);
	}
}

K_THREAD_DEFINE(mqtt_thread_id, 2048, mqtt_thread, NULL, NULL, NULL, 7, 0, 0);
