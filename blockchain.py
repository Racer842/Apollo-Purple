#!/usr/bin/env python3
"""
MQTT Server with IOTA Blockchain Integration
Receives sensor data from ESP32 devices and stores on IOTA Tangle
"""

import json
import time
import logging
import asyncio
from datetime import datetime
from typing import Dict, Any, Optional
import paho.mqtt.client as mqtt
import sqlite3
from dataclasses import dataclass, asdict
import threading
import signal
import sys
from iota_sdk import Client, HexStr, utf8_to_hex

# IOTA libraries
try:
    from iota_sdk import Client, utf8_to_hex, Utils
    IOTA_AVAILABLE = True
except ImportError:
    print("Warning: IOTA SDK not available. Install with: pip install iota-sdk")
    IOTA_AVAILABLE = False

# Configuration
MQTT_CONFIG = {
    'broker_host': 'localhost',
    'broker_port': 1883,
    'topics': {
        'core2': 'sensor/core2/+',
        'esp32c3': 'sensor/esp32c3/+',
        'commands': 'commands/+'
    },
    'client_id': 'mqtt_iota_bridge'
}

IOTA_CONFIG = {
    'node_url': 'https://api.testnet.shimmer.network',  # Shimmer testnet
    'network': 'testnet'
}

DATABASE_CONFIG = {
    'db_path': 'sensor_data.db'
}

@dataclass
class SensorReading:
    device_id: str
    sensor_type: str
    value: float
    unit: str
    timestamp: str
    location: Optional[str] = None

class DatabaseManager:
    def __init__(self, db_path: str):
        self.db_path = db_path
        self.init_database()
    
    def init_database(self):
        """Initialize SQLite database with required tables"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Sensor data table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS sensor_readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                device_id TEXT NOT NULL,
                sensor_type TEXT NOT NULL,
                value REAL NOT NULL,
                unit TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                location TEXT,
                iota_message_id TEXT,
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # IOTA transaction log
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS iota_transactions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                message_id TEXT UNIQUE NOT NULL,
                block_id TEXT,
                data_hash TEXT,
                timestamp TEXT NOT NULL,
                status TEXT DEFAULT 'pending',
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        conn.commit()
        conn.close()
    
    def store_sensor_data(self, reading: SensorReading, iota_message_id: Optional[str] = None):
        """Store sensor reading in database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            INSERT INTO sensor_readings 
            (device_id, sensor_type, value, unit, timestamp, location, iota_message_id)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (
            reading.device_id, reading.sensor_type, reading.value,
            reading.unit, reading.timestamp, reading.location, iota_message_id
        ))
        
        conn.commit()
        conn.close()
    
    def store_iota_transaction(self, message_id: str, data_hash: str, timestamp: str):
        """Store IOTA transaction details"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            INSERT OR REPLACE INTO iota_transactions 
            (message_id, data_hash, timestamp)
            VALUES (?, ?, ?)
        ''', (message_id, data_hash, timestamp))
        
        conn.commit()
        conn.close()
    
    def get_recent_readings(self, limit: int = 100) -> list:
        """Get recent sensor readings"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT * FROM sensor_readings 
            ORDER BY created_at DESC 
            LIMIT ?
        ''', (limit,))
        
        results = cursor.fetchall()
        conn.close()
        return results

class IOTAManager:
    def __init__(self, config: dict):
        self.config = config
        self.client = None
        self.initialized = False
        
        if IOTA_AVAILABLE:
            self.init_client()
    
    def init_client(self):
        """Initialize IOTA client"""
        try:
            self.client = Client(nodes=[self.config['node_url']])
            self.initialized = True
            logging.info("IOTA client initialized successfully")
        except Exception as e:
            logging.error(f"Failed to initialize IOTA client: {e}")
            self.initialized = False
    
    async def send_to_tangle(self, data: dict) -> Optional[str]:
        """Send data to IOTA Tangle"""
        if not self.initialized:
            logging.warning("IOTA client not initialized")
            return None
        
        try:
            # Convert data to JSON string and then to hex
            json_data = json.dumps(data, indent=2)
            hex_data = utf8_to_hex(json_data)
            
            # Send message to Tangle
            message = await self.client.build_and_post_block(data=hex_data)
            
            if message and hasattr(message, 'id'):
                logging.info(f"Data sent to IOTA Tangle. Message ID: {message.id}")
                return message.id
            else:
                logging.error("Failed to get message ID from IOTA response")
                return None
                
        except Exception as e:
            logging.error(f"Error sending data to IOTA Tangle: {e}")
            return None

class MQTTIOTABridge:
    def __init__(self):
        self.db_manager = DatabaseManager(DATABASE_CONFIG['db_path'])
        self.iota_manager = IOTAManager(IOTA_CONFIG)
        self.mqtt_client = None
        self.running = False
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('mqtt_iota_bridge.log'),
                logging.StreamHandler()
            ]
        )
        
        self.setup_mqtt_client()
    
    def setup_mqtt_client(self):
        """Setup MQTT client with callbacks"""
        self.mqtt_client = mqtt.Client(client_id=MQTT_CONFIG['client_id'])
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logging.info("Connected to MQTT broker")
            # Subscribe to all sensor topics
            for topic in MQTT_CONFIG['topics'].values():
                client.subscribe(topic)
                logging.info(f"Subscribed to topic: {topic}")
        else:
            logging.error(f"Failed to connect to MQTT broker. Return code: {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        logging.info("Disconnected from MQTT broker")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            logging.info(f"Received message on topic {topic}: {payload}")
            
            # Parse the message
            if topic.startswith('sensor/'):
                self.handle_sensor_data(topic, payload)
            elif topic.startswith('commands/'):
                self.handle_command(topic, payload)
                
        except Exception as e:
            logging.error(f"Error processing MQTT message: {e}")
    
    def handle_sensor_data(self, topic: str, payload: str):
        """Handle sensor data messages"""
        try:
            # Parse topic to extract device and sensor info
            topic_parts = topic.split('/')
            if len(topic_parts) >= 3:
                device_id = topic_parts[1]  # core2 or esp32c3
                sensor_type = topic_parts[2]  # temperature, humidity, etc.
            else:
                device_id = "unknown"
                sensor_type = "unknown"
            
            # Parse payload (assuming JSON format)
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                # If not JSON, treat as simple value
                data = {"value": float(payload), "unit": "unknown"}
            
            # Create sensor reading
            reading = SensorReading(
                device_id=device_id,
                sensor_type=sensor_type,
                value=float(data.get('value', 0)),
                unit=data.get('unit', 'unknown'),
                timestamp=data.get('timestamp', datetime.now().isoformat()),
                location=data.get('location')
            )
            
            # Store in database and send to IOTA
            asyncio.create_task(self.process_sensor_reading(reading))
            
        except Exception as e:
            logging.error(f"Error handling sensor data: {e}")
    
    async def process_sensor_reading(self, reading: SensorReading):
        """Process sensor reading - store in DB and send to IOTA"""
        try:
            # Prepare data for IOTA
            iota_data = {
                "type": "sensor_reading",
                "device_id": reading.device_id,
                "sensor_type": reading.sensor_type,
                "value": reading.value,
                "unit": reading.unit,
                "timestamp": reading.timestamp,
                "location": reading.location
            }
            
            # Send to IOTA Tangle
            message_id = await self.iota_manager.send_to_tangle(iota_data)
            
            # Store in database
            self.db_manager.store_sensor_data(reading, message_id)
            
            if message_id:
                # Store IOTA transaction
                data_hash = str(hash(json.dumps(iota_data)))
                self.db_manager.store_iota_transaction(
                    message_id, data_hash, reading.timestamp
                )
                logging.info(f"Sensor data processed and stored on IOTA: {message_id}")
            else:
                logging.warning("Failed to store data on IOTA, but saved locally")
                
        except Exception as e:
            logging.error(f"Error processing sensor reading: {e}")
    
    def handle_command(self, topic: str, payload: str):
        """Handle command messages"""
        try:
            topic_parts = topic.split('/')
            if len(topic_parts) >= 2:
                device_id = topic_parts[1]
                
                # Forward command to specific device
                response_topic = f"response/{device_id}"
                self.mqtt_client.publish(response_topic, payload)
                logging.info(f"Command forwarded to {device_id}: {payload}")
                
        except Exception as e:
            logging.error(f"Error handling command: {e}")
    
    def publish_test_data(self):
        """Publish test sensor data for debugging"""
        test_data = {
            "value": 25.5,
            "unit": "°C",
            "timestamp": datetime.now().isoformat(),
            "location": "test_lab"
        }
        
        self.mqtt_client.publish("sensor/core2/temperature", json.dumps(test_data))
        logging.info("Published test data")
    
    def start(self):
        """Start the MQTT-IOTA bridge"""
        self.running = True
        
        try:
            # Connect to MQTT broker
            self.mqtt_client.connect(
                MQTT_CONFIG['broker_host'], 
                MQTT_CONFIG['broker_port'], 
                60
            )
            
            # Start MQTT loop in background thread
            self.mqtt_client.loop_start()
            
            logging.info("MQTT-IOTA Bridge started")
            
            # Keep the main thread alive
            while self.running:
                time.sleep(1)
                
        except KeyboardInterrupt:
            logging.info("Received interrupt signal")
        except Exception as e:
            logging.error(f"Error in main loop: {e}")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the MQTT-IOTA bridge"""
        self.running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        logging.info("MQTT-IOTA Bridge stopped")

def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown"""
    print("\nReceived interrupt signal. Shutting down...")
    sys.exit(0)

def main():
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and start the bridge
    bridge = MQTTIOTABridge()
    
    print("Starting MQTT-IOTA Bridge...")
    print("Press Ctrl+C to stop")
    
    bridge.start()

if __name__ == "__main__":
    main()