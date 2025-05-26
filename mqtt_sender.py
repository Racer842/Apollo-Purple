import json
import time
from datetime import datetime
import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point, WritePrecision

# --------------------------- Configuration ---------------------------

# MQTT
MQTT_BROKER = "172.20.10.11"
MQTT_PORT = 1883
C3_TOPIC = "esp32c3/sensors"
CORE2_TOPIC = "m5core2/temp"

# InfluxDB
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "F5UpKMQW8ooHbOsvAbkg69hfxv-r270T4N0HO7H-S_ooObPF_R3_XM53mBdeuntoOvl9BFGBXPZwcNMX3-_hoQ=="
INFLUX_ORG = "CSSE_4011"
INFLUX_BUCKET = "sensor_data"

# --------------------------- InfluxDB Setup ---------------------------

influx_client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG
)
influx_write_api = influx_client.write_api(write_options=None)

# --------------------------- MQTT Callbacks ---------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(C3_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print(f"Received message on {msg.topic}: {json.dumps(payload, indent=2)}")

        timestamp = datetime.utcnow()

        # Store each Thingy52 set in InfluxDB
        for idx, thing in enumerate(payload.get("thingy52", [])):
            point = (
                Point(f"thingy52_{idx}")
                .field("temperature", thing["temp"])
                .field("humidity", thing["humidity"])
                .field("co2", thing["co2"])
                .time(timestamp, WritePrecision.NS)
            )
            influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)

        # Moisture sensor data
        point_moisture = (
            Point("moisture")
            .field("moisture1", payload.get("moisture1", 0))
            .field("moisture2", payload.get("moisture2", 0))
            .time(timestamp, WritePrecision.NS)
        )
        influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point_moisture)

        # Prepare and publish temperature data for Core2
        temps = [t["temp"] for t in payload.get("thingy52", [])]
        avg_temp = sum(temps) / len(temps) if temps else 0

        core2_payload = json.dumps({"average_temp": avg_temp})
        client.publish(CORE2_TOPIC, core2_payload)
        print(f"Published average temperature to {CORE2_TOPIC}: {core2_payload}")

    except Exception as e:
        print(f"Error handling message: {e}")

# --------------------------- MQTT Setup ---------------------------

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)

print("Starting MQTT loop...")
client.loop_forever()
