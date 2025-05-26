#!/usr/bin/env python3
"""
MQTT Message Sender for M5 Core2
Sends periodic messages to the M5 Core2 device via MQTT broker
"""

import paho.mqtt.client as mqtt
import time
import json
import random
from datetime import datetime

# MQTT Configuration - Update with your Mac's IP if different
MQTT_BROKER_HOST = "172.20.10.11"  # Your Mac's IP from the C code
MQTT_BROKER_PORT = 1883
MQTT_CLIENT_ID = "python_sender"
MQTT_TOPIC_COMMAND = "m5core2/command"  # Topic the M5 Core2 is listening on
MQTT_TOPIC_STATUS = "m5core2/status"    # Topic the M5 Core2 publishes to

# Message sending interval (seconds)
MESSAGE_INTERVAL = 5

# Global variables
client = None
message_counter = 0

def on_connect(client, userdata, flags, rc):
    """Callback for when the client receives a CONNACK response from the server."""
    if rc == 0:
        print(f"✅ Connected to MQTT broker at {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
        # Subscribe to status topic to see responses from M5 Core2
        client.subscribe(MQTT_TOPIC_STATUS)
        print(f"📡 Subscribed to {MQTT_TOPIC_STATUS}")
    else:
        print(f"❌ Failed to connect to MQTT broker. Return code: {rc}")

def on_disconnect(client, userdata, rc):
    """Callback for when the client disconnects from the server."""
    print(f"🔌 Disconnected from MQTT broker. Return code: {rc}")

def on_message(client, userdata, msg):
    """Callback for when a PUBLISH message is received from the server."""
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"📨 [{timestamp}] Received from {topic}: {payload}")

def on_publish(client, userdata, mid):
    """Callback for when a message is published."""
    print(f"📤 Message published successfully (message ID: {mid})")

def create_test_messages():
    """Create a variety of test messages to send to the M5 Core2."""
    messages = [
        "Hello from Python!",
        "Testing MQTT communication",
        "How are you doing, M5 Core2?",
        "This is a test message",
        "Greetings from your Mac!",
        "MQTT is working great!",
        "Keep up the good work!",
        "Python says hello 👋",
        "Time to shine, little device!",
        "Another message from the mothership",
    ]
    
    # Add some dynamic messages
    timestamp = datetime.now().strftime("%H:%M:%S")
    messages.extend([
        f"Current time: {timestamp}",
        f"Random number: {random.randint(1, 100)}",
        f"Message #{message_counter + 1}",
    ])
    
    return messages

def send_message():
    """Send a message to the M5 Core2."""
    global message_counter
    
    if not client or not client.is_connected():
        print("❌ MQTT client not connected")
        return False
    
    messages = create_test_messages()
    message = random.choice(messages)
    
    try:
        # Send the message
        result = client.publish(MQTT_TOPIC_COMMAND, message, qos=0)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            message_counter += 1
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"📝 [{timestamp}] Sent to M5 Core2: '{message}'")
            return True
        else:
            print(f"❌ Failed to send message. Error code: {result.rc}")
            return False
            
    except Exception as e:
        print(f"❌ Error sending message: {e}")
        return False

def send_json_message():
    """Send a JSON formatted message for more structured communication."""
    global message_counter
    
    if not client or not client.is_connected():
        print("❌ MQTT client not connected")
        return False
    
    # Create a JSON message
    json_msg = {
        "id": message_counter + 1,
        "timestamp": datetime.now().isoformat(),
        "sender": "Python Script",
        "message": f"JSON message #{message_counter + 1}",
        "data": {
            "temperature": round(random.uniform(20.0, 30.0), 1),
            "humidity": random.randint(40, 80),
            "status": "active"
        }
    }
    
    try:
        json_string = json.dumps(json_msg, indent=None)
        result = client.publish(MQTT_TOPIC_COMMAND, json_string, qos=0)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            message_counter += 1
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"📋 [{timestamp}] Sent JSON to M5 Core2: {json_string}")
            return True
        else:
            print(f"❌ Failed to send JSON message. Error code: {result.rc}")
            return False
            
    except Exception as e:
        print(f"❌ Error sending JSON message: {e}")
        return False

def main():
    """Main function to run the MQTT message sender."""
    global client
    
    print("🚀 Starting MQTT Message Sender for M5 Core2")
    print(f"📡 Broker: {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
    print(f"📨 Sending to topic: {MQTT_TOPIC_COMMAND}")
    print(f"📢 Listening to topic: {MQTT_TOPIC_STATUS}")
    print(f"⏰ Message interval: {MESSAGE_INTERVAL} seconds")
    print("-" * 50)
    
    # Create and configure MQTT client
    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.on_publish = on_publish
    
    try:
        # Connect to the broker
        print(f"🔄 Connecting to MQTT broker...")
        client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
        
        # Start the network loop in a separate thread
        client.loop_start()
        
        # Wait a moment for connection
        time.sleep(2)
        
        if not client.is_connected():
            print("❌ Failed to connect to MQTT broker. Check your broker settings.")
            return
        
        print("✅ Ready to send messages! Press Ctrl+C to stop.")
        print("-" * 50)
        
        # Main message sending loop
        message_count = 0
        while True:
            # Alternate between regular and JSON messages
            if message_count % 4 == 3:  # Every 4th message is JSON
                send_json_message()
            else:
                send_message()
            
            message_count += 1
            
            # Wait for the specified interval
            time.sleep(MESSAGE_INTERVAL)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping message sender...")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        if client:
            client.loop_stop()
            client.disconnect()
            print("👋 Disconnected from MQTT broker. Goodbye!")

if __name__ == "__main__":
    main()
