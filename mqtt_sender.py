import json
import time
import hashlib
import struct
from datetime import datetime
from typing import Dict, List, Any, Optional
import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point, WritePrecision
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
import base64
import os

# --------------------------- Configuration ---------------------------

# MQTT
MQTT_BROKER = "172.20.10.11"
MQTT_PORT = 1883
C3_TOPIC = "esp32c3/sensors"
CORE2_TOPIC = "m5core2/temp"
BLOCKCHAIN_TOPIC = "blockchain/sensor_data"

# InfluxDB
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "F5UpKMQW8ooHbOsvAbkg69hfxv-r270T4N0HO7H-S_ooObPF_R3_XM53mBdeuntoOvl9BFGBXPZwcNMX3-_hoQ=="
INFLUX_ORG = "CSSE_4011"
INFLUX_BUCKET = "sensor_data"
INFLUX_BLOCKCHAIN_BUCKET = "blockchain_data"

# Blockchain Configuration
BLOCKCHAIN_PASSWORD = "your_secure_password_here"  # Change this!
GENESIS_HASH = "0000000000000000000000000000000000000000000000000000000000000000"

# --------------------------- Blockchain Classes ---------------------------

class Block:
    def __init__(self, index: int, timestamp: float, data: Dict[str, Any], 
                 previous_hash: str, nonce: int = 0):
        self.index = index
        self.timestamp = timestamp
        self.data = data
        self.previous_hash = previous_hash
        self.nonce = nonce
        self.hash = self.calculate_hash()

    def calculate_hash(self) -> str:
        """Calculate SHA-256 hash of the block"""
        block_string = json.dumps({
            "index": self.index,
            "timestamp": self.timestamp,
            "data": self.data,
            "previous_hash": self.previous_hash,
            "nonce": self.nonce
        }, sort_keys=True)
        return hashlib.sha256(block_string.encode()).hexdigest()

    def mine_block(self, difficulty: int = 4):
        """Simple proof-of-work mining"""
        target = "0" * difficulty
        while self.hash[:difficulty] != target:
            self.nonce += 1
            self.hash = self.calculate_hash()
        print(f"Block mined: {self.hash}")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "index": self.index,
            "timestamp": self.timestamp,
            "data": self.data,
            "previous_hash": self.previous_hash,
            "nonce": self.nonce,
            "hash": self.hash
        }

class Blockchain:
    def __init__(self):
        self.chain: List[Block] = [self.create_genesis_block()]
        self.difficulty = 2  # Adjust for faster/slower mining

    def create_genesis_block(self) -> Block:
        return Block(0, time.time(), {"genesis": "block"}, GENESIS_HASH)

    def get_latest_block(self) -> Block:
        return self.chain[-1]

    def add_block(self, data: Dict[str, Any]):
        previous_block = self.get_latest_block()
        new_block = Block(
            index=previous_block.index + 1,
            timestamp=time.time(),
            data=data,
            previous_hash=previous_block.hash
        )
        new_block.mine_block(self.difficulty)
        self.chain.append(new_block)
        return new_block

    def is_chain_valid(self) -> bool:
        for i in range(1, len(self.chain)):
            current_block = self.chain[i]
            previous_block = self.chain[i-1]

            if current_block.hash != current_block.calculate_hash():
                return False

            if current_block.previous_hash != previous_block.hash:
                return False

        return True

# --------------------------- Encryption Utilities ---------------------------

class DataEncryption:
    def __init__(self, password: str):
        self.password = password.encode()
        self.salt = os.urandom(16)
        self.key = self._derive_key()
        self.cipher_suite = Fernet(self.key)

    def _derive_key(self) -> bytes:
        kdf = PBKDF2HMAC(
            algorithm=hashes.SHA256(),
            length=32,
            salt=self.salt,
            iterations=100000,
        )
        key = base64.urlsafe_b64encode(kdf.derive(self.password))
        return key

    def encrypt_data(self, data: Dict[str, Any]) -> str:
        """Encrypt JSON data and return base64 encoded string"""
        json_data = json.dumps(data, sort_keys=True)
        encrypted_data = self.cipher_suite.encrypt(json_data.encode())
        return base64.b64encode(encrypted_data).decode()

    def decrypt_data(self, encrypted_data: str) -> Dict[str, Any]:
        """Decrypt base64 encoded data and return JSON"""
        encrypted_bytes = base64.b64decode(encrypted_data.encode())
        decrypted_data = self.cipher_suite.decrypt(encrypted_bytes)
        return json.loads(decrypted_data.decode())

# --------------------------- Nanopb Protocol Buffer Handling ---------------------------

class NanoPBHandler:
    """
    Simulated nanopb handler for Thingy52 sensor data
    In real implementation, this would use actual protobuf definitions
    """
    
    @staticmethod
    def decode_thingy52_data(raw_data: bytes) -> Dict[str, Any]:
        """
        Decode nanopb formatted Thingy52 data
        This is a simplified implementation - replace with actual nanopb decoding
        """
        try:
            # Assuming data format: [temp(float), humidity(float), co2(uint16), pressure(float)]
            if len(raw_data) >= 14:  # 4+4+2+4 bytes
                temp = struct.unpack('<f', raw_data[0:4])[0]
                humidity = struct.unpack('<f', raw_data[4:8])[0]
                co2 = struct.unpack('<H', raw_data[8:10])[0]
                pressure = struct.unpack('<f', raw_data[10:14])[0]
                
                return {
                    "temperature": round(temp, 2),
                    "humidity": round(humidity, 2),
                    "co2": co2,
                    "pressure": round(pressure, 2),
                    "timestamp": datetime.utcnow().isoformat()
                }
            else:
                raise ValueError("Insufficient data length")
        except Exception as e:
            print(f"Error decoding nanopb data: {e}")
            return {}

    @staticmethod
    def simulate_thingy52_data() -> bytes:
        """Generate simulated Thingy52 data for testing"""
        import random
        temp = random.uniform(20.0, 30.0)
        humidity = random.uniform(30.0, 80.0)
        co2 = random.randint(400, 1200)
        pressure = random.uniform(990.0, 1030.0)
        
        return struct.pack('<ffHf', temp, humidity, co2, pressure)

# --------------------------- Main Application Class ---------------------------

class Thingy52DataHandler:
    def __init__(self):
        # Initialize components
        self.blockchain = Blockchain()
        self.encryption = DataEncryption(BLOCKCHAIN_PASSWORD)
        self.nanopb_handler = NanoPBHandler()
        
        # InfluxDB setup
        self.influx_client = InfluxDBClient(
            url=INFLUX_URL,
            token=INFLUX_TOKEN,
            org=INFLUX_ORG
        )
        self.influx_write_api = self.influx_client.write_api(write_options=None)
        
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        # Data storage
        self.sensor_data_buffer = []

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(C3_TOPIC)
            client.subscribe("thingy52/raw_data")  # Subscribe to raw nanopb data
        else:
            print(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == "thingy52/raw_data":
                # Handle raw nanopb data
                self.process_nanopb_data(msg.payload)
            else:
                # Handle JSON data (existing format)
                payload = json.loads(msg.payload.decode())
                self.process_json_data(payload)
        except Exception as e:
            print(f"Error handling message: {e}")

    def process_nanopb_data(self, raw_data: bytes):
        """Process raw nanopb data from Thingy52"""
        decoded_data = self.nanopb_handler.decode_thingy52_data(raw_data)
        if decoded_data:
            print(f"Decoded Thingy52 data: {decoded_data}")
            
            # Store in InfluxDB
            self.store_sensor_data_influx([decoded_data])
            
            # Add to blockchain
            self.add_to_blockchain(decoded_data)
            
            # Publish processed data
            self.publish_processed_data([decoded_data])

    def process_json_data(self, payload: Dict[str, Any]):
        """Process JSON formatted data (existing format)"""
        print(f"Received JSON data: {json.dumps(payload, indent=2)}")
        
        thingy_data = payload.get("thingy52", [])
        if thingy_data:
            # Store in InfluxDB
            self.store_sensor_data_influx(thingy_data)
            
            # Add to blockchain
            for data in thingy_data:
                self.add_to_blockchain(data)
            
            # Publish processed data
            self.publish_processed_data(thingy_data)
        
        # Handle moisture data
        if "moisture1" in payload or "moisture2" in payload:
            self.store_moisture_data_influx(payload)

    def store_sensor_data_influx(self, sensor_data: List[Dict[str, Any]]):
        """Store sensor data in InfluxDB"""
        timestamp = datetime.utcnow()
        
        for idx, data in enumerate(sensor_data):
            point = (
                Point(f"thingy52_{idx}")
                .field("temperature", data.get("temp", data.get("temperature", 0)))
                .field("humidity", data.get("humidity", 0))
                .field("co2", data.get("co2", 0))
                .field("pressure", data.get("pressure", 0))
                .time(timestamp, WritePrecision.NS)
            )
            self.influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)

    def store_moisture_data_influx(self, payload: Dict[str, Any]):
        """Store moisture sensor data in InfluxDB"""
        timestamp = datetime.utcnow()
        point_moisture = (
            Point("moisture")
            .field("moisture1", payload.get("moisture1", 0))
            .field("moisture2", payload.get("moisture2", 0))
            .time(timestamp, WritePrecision.NS)
        )
        self.influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point_moisture)

    def add_to_blockchain(self, sensor_data: Dict[str, Any]):
        """Add sensor data to blockchain with encryption"""
        try:
            # Encrypt the sensor data
            encrypted_data = self.encryption.encrypt_data(sensor_data)
            
            # Create blockchain entry
            blockchain_data = {
                "sensor_id": "thingy52",
                "encrypted_payload": encrypted_data,
                "timestamp": datetime.utcnow().isoformat(),
                "data_hash": hashlib.sha256(json.dumps(sensor_data, sort_keys=True).encode()).hexdigest()
            }
            
            # Add to blockchain
            new_block = self.blockchain.add_block(blockchain_data)
            
            # Store blockchain data in InfluxDB for Grafana access
            self.store_blockchain_data_influx(new_block)
            
            # Publish blockchain event
            self.mqtt_client.publish(BLOCKCHAIN_TOPIC, json.dumps({
                "block_index": new_block.index,
                "block_hash": new_block.hash,
                "timestamp": new_block.timestamp
            }))
            
            print(f"Added block {new_block.index} to blockchain: {new_block.hash[:16]}...")
            
        except Exception as e:
            print(f"Error adding to blockchain: {e}")

    def store_blockchain_data_influx(self, block: Block):
        """Store blockchain metadata in InfluxDB for Grafana visualization"""
        timestamp = datetime.utcnow()
        point = (
            Point("blockchain")
            .field("block_index", block.index)
            .field("block_hash", block.hash)
            .field("previous_hash", block.previous_hash)
            .field("nonce", block.nonce)
            .field("data_size", len(json.dumps(block.data)))
            .time(timestamp, WritePrecision.NS)
        )
        self.influx_write_api.write(bucket=INFLUX_BLOCKCHAIN_BUCKET, org=INFLUX_ORG, record=point)

    def publish_processed_data(self, sensor_data: List[Dict[str, Any]]):
        """Publish processed temperature data to Core2"""
        temps = [data.get("temp", data.get("temperature", 0)) for data in sensor_data]
        avg_temp = sum(temps) / len(temps) if temps else 0

        core2_payload = json.dumps({"average_temp": avg_temp})
        self.mqtt_client.publish(CORE2_TOPIC, core2_payload)
        print(f"Published average temperature to {CORE2_TOPIC}: {core2_payload}")

    def decrypt_blockchain_data(self, block_index: int) -> Optional[Dict[str, Any]]:
        """Decrypt sensor data from a specific blockchain block"""
        if 0 <= block_index < len(self.blockchain.chain):
            block = self.blockchain.chain[block_index]
            encrypted_payload = block.data.get("encrypted_payload")
            if encrypted_payload:
                try:
                    return self.encryption.decrypt_data(encrypted_payload)
                except Exception as e:
                    print(f"Error decrypting block {block_index}: {e}")
        return None

    # # Commented out moisture sensor functionality for future use
    # def publish_moisture_data(self, moisture1: float, moisture2: float):
    #     """Publish moisture sensor data"""
    #     moisture_payload = {
    #         "moisture1": moisture1,
    #         "moisture2": moisture2,
    #         "timestamp": datetime.utcnow().isoformat()
    #     }
    #     
    #     # Store in InfluxDB
    #     self.store_moisture_data_influx(moisture_payload)
    #     
    #     # Add to blockchain
    #     self.add_to_blockchain(moisture_payload)
    #     
    #     # Publish to MQTT
    #     self.mqtt_client.publish("sensors/moisture", json.dumps(moisture_payload))
    #     print(f"Published moisture data: {moisture_payload}")

    def start(self):
        """Start the data handler"""
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        print("Starting Thingy52 Data Handler with Blockchain...")
        print(f"Blockchain initialized with {len(self.blockchain.chain)} blocks")
        
        # Start MQTT loop in a separate thread for testing
        self.mqtt_client.loop_start()
        
        return self.mqtt_client

    def simulate_data_flow(self):
        """Simulate Thingy52 data for testing"""
        while True:
            try:
                # Simulate nanopb data
                raw_data = self.nanopb_handler.simulate_thingy52_data()
                self.process_nanopb_data(raw_data)
                
                time.sleep(10)  # Send data every 10 seconds
            except KeyboardInterrupt:
                print("Stopping simulation...")
                break

# --------------------------- Main Execution ---------------------------

if __name__ == "__main__":
    handler = Thingy52DataHandler()
    
    # Start the handler
    mqtt_client = handler.start()
    
    try:
        # For testing: simulate data flow
        print("Starting data simulation (Press Ctrl+C to stop)...")
        handler.simulate_data_flow()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        handler.influx_client.close()