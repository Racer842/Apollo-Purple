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
INFLUX_TOKEN = "srrM5ABK0NPQXPjnEv9hu2Q2vlyMC8W15OPBc8dnEeSTw_kZt82RSUgSsBtFp3-hN0CK-jqeDZDtkFI5v9DKEA=="
INFLUX_ORG = "CSSE_4011"
INFLUX_BUCKET = "sensor_data"
INFLUX_BLOCKCHAIN_BUCKET = "blockchain_data"
# NEW: Separate bucket for blockchain sensor data
INFLUX_BLOCKCHAIN_SENSOR_BUCKET = "blockchain_sensor_data"

# Blockchain Configuration
BLOCKCHAIN_PASSWORD = "csse_4011_project"  # Change this!
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
        # Use a fixed salt for consistent key generation (for demo purposes)
        # In production, you'd want to store this securely
        self.salt = b'fixed_salt_123456'  # Changed to fixed salt for consistent decryption
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
        temp = random.uniform(20.0, 35.0)
        humidity = random.uniform(30.0, 80.0)
        co2 = random.randint(400, 1200)
        pressure = random.uniform(990.0, 1030.0)
        
        return struct.pack('<ffHf', temp, humidity, co2, pressure)

    @staticmethod
    def simulate_multiple_thingy52_data(count: int = 4) -> List[bytes]:
        """Generate multiple simulated Thingy52 data for testing"""
        return [NanoPBHandler.simulate_thingy52_data() for _ in range(count)]

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
        self.influx_write_api = self.influx_client.write_api()
        
        # MQTT setup
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        # Data storage
        self.sensor_data_buffer = []

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
            client.subscribe(C3_TOPIC)
            client.subscribe("thingy52/raw_data")  # Subscribe to raw nanopb data
            client.subscribe("thingy52/multiple_data")  # Subscribe to multiple sensor data
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == "thingy52/raw_data":
                # Handle raw nanopb data
                self.process_nanopb_data(msg.payload)
            elif msg.topic == "thingy52/multiple_data":
                # Handle multiple sensor data JSON
                payload = json.loads(msg.payload.decode())
                self.process_multiple_sensor_data(payload)
            else:
                # Handle JSON data (existing format)
                payload = json.loads(msg.payload.decode())
                self.process_json_data(payload)
        except Exception as e:
            print(f"Error handling message: {e}")

    def process_multiple_sensor_data(self, payload: Dict[str, Any]):
        """Process data from multiple sensors"""
        print(f"Received multiple sensor data: {json.dumps(payload, indent=2)}")
        
        sensors_data = payload.get("sensors", [])
        if sensors_data:
            # Store in InfluxDB
            self.store_multiple_sensor_data_influx(sensors_data)
            
            # Add each sensor reading to blockchain
            for data in sensors_data:
                self.add_to_blockchain(data)
            
            # Publish 4 temperature readings to Core2
            self.publish_four_temperatures(sensors_data)

    def process_nanopb_data(self, raw_data: bytes):
        """Process raw nanopb data from Thingy52"""
        decoded_data = self.nanopb_handler.decode_thingy52_data(raw_data)
        if decoded_data:
            print(f"Decoded Thingy52 data: {decoded_data}")
            
            # Store in regular InfluxDB (for comparison)
            self.store_sensor_data_influx([decoded_data])
            
            # Add to blockchain AND store decrypted data for Grafana
            self.add_to_blockchain(decoded_data)
            
            # For single sensor, create 4 variations and publish
            self.publish_single_temp_as_four(decoded_data)

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
            
            # Publish 4 temperature readings
            self.publish_four_temperatures(thingy_data)
        
        # Handle moisture data
        if "moisture1" in payload or "moisture2" in payload:
            self.store_moisture_data_influx(payload)

    def store_multiple_sensor_data_influx(self, sensors_data: List[Dict[str, Any]]):
        """Store multiple sensor data in InfluxDB"""
        timestamp = datetime.utcnow()
        
        for idx, data in enumerate(sensors_data):
            point = (
                Point(f"thingy52_sensor_{idx}")
                .field("temperature", data.get("temp", data.get("temperature", 0)))
                .field("humidity", data.get("humidity", 0))
                .field("co2", data.get("co2", 0))
                .field("pressure", data.get("pressure", 0))
                .field("sensor_id", idx)
                .time(timestamp, WritePrecision.NS)
            )
            self.influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)

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

    def publish_four_temperatures(self, sensor_data: List[Dict[str, Any]]):
        """Publish 4 temperature readings to Core2"""
        temperatures = []
        
        # Extract up to 4 temperature readings
        for i, data in enumerate(sensor_data[:4]):
            temp = data.get("temp", data.get("temperature", 0))
            temperatures.append(round(temp, 1))
        
        # If we have fewer than 4 readings, pad with variations of the last reading
        while len(temperatures) < 4:
            if temperatures:
                # Create slight variations of the last temperature
                base_temp = temperatures[-1]
                variation = round(base_temp + (len(temperatures) - 1) * 0.5, 1)
                temperatures.append(variation)
            else:
                temperatures.append(25.0)  # Default temperature
        
        # Create payload with 4 temperature readings
        core2_payload = {
            "temp1": temperatures[0],
            "temp2": temperatures[1], 
            "temp3": temperatures[2],
            "temp4": temperatures[3],
            "timestamp": datetime.utcnow().isoformat()
        }

        payload_json = json.dumps(core2_payload)
        self.mqtt_client.publish(CORE2_TOPIC, payload_json)
        print(f"Published 4 temperatures to {CORE2_TOPIC}: {payload_json}")

    def publish_single_temp_as_four(self, sensor_data: Dict[str, Any]):
        """Convert single temperature reading into 4 readings with slight variations"""
        base_temp = sensor_data.get("temp", sensor_data.get("temperature", 25.0))
        
        # Create 4 temperature readings with variations
        temperatures = [
            round(base_temp, 1),
            round(base_temp + 1.5, 1),
            round(base_temp - 0.5, 1),
            round(base_temp + 2.0, 1)
        ]
        
        core2_payload = {
            "temp1": temperatures[0],
            "temp2": temperatures[1],
            "temp3": temperatures[2], 
            "temp4": temperatures[3],
            "timestamp": datetime.utcnow().isoformat()
        }

        payload_json = json.dumps(core2_payload)
        self.mqtt_client.publish(CORE2_TOPIC, payload_json)
        print(f"Published 4 temperature variations to {CORE2_TOPIC}: {payload_json}")

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
            
            # Store blockchain metadata in InfluxDB
            self.store_blockchain_data_influx(new_block)
            
            # NEW: Store decrypted sensor data with blockchain reference for Grafana
            self.store_blockchain_sensor_data_influx(new_block, sensor_data)
            
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
            Point("blockchain_metadata")
            .field("block_index", block.index)
            .field("block_hash", block.hash)
            .field("previous_hash", block.previous_hash)
            .field("nonce", block.nonce)
            .field("data_size", len(json.dumps(block.data)))
            .field("mining_timestamp", block.timestamp)
            .time(timestamp, WritePrecision.NS)
        )
        self.influx_write_api.write(bucket=INFLUX_BLOCKCHAIN_BUCKET, org=INFLUX_ORG, record=point)

    def store_blockchain_sensor_data_influx(self, block: Block, original_sensor_data: Dict[str, Any]):
        """NEW: Store decrypted sensor data with blockchain reference for Grafana access"""
        timestamp = datetime.utcnow()
        
        # Store the actual sensor values with blockchain metadata
        point = (
            Point("blockchain_sensor_data")
            .tag("block_index", str(block.index))
            .tag("block_hash", block.hash[:16])  # Shortened hash for tag
            .tag("sensor_id", "thingy52")
            .field("temperature", original_sensor_data.get("temp", original_sensor_data.get("temperature", 0)))
            .field("humidity", original_sensor_data.get("humidity", 0))
            .field("co2", original_sensor_data.get("co2", 0))
            .field("pressure", original_sensor_data.get("pressure", 0))
            .field("block_index_field", block.index)  # Also as field for easier querying
            .field("data_integrity_hash", block.data.get("data_hash", ""))
            .field("is_blockchain_verified", True)
            .time(timestamp, WritePrecision.NS)
        )
        self.influx_write_api.write(bucket=INFLUX_BLOCKCHAIN_SENSOR_BUCKET, org=INFLUX_ORG, record=point)
        
        print(f"Stored blockchain sensor data for block {block.index}")

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

    def get_blockchain_summary(self) -> Dict[str, Any]:
        """Get blockchain summary for monitoring"""
        return {
            "total_blocks": len(self.blockchain.chain),
            "latest_block_hash": self.blockchain.get_latest_block().hash,
            "blockchain_valid": self.blockchain.is_chain_valid(),
            "genesis_block": self.blockchain.chain[0].hash
        }

    # NEW: Method to recreate blockchain sensor data from existing blockchain
    def rebuild_blockchain_sensor_data(self):
        """Rebuild the blockchain sensor data in InfluxDB from existing blockchain"""
        print("Rebuilding blockchain sensor data from existing blockchain...")
        for block in self.blockchain.chain[1:]:  # Skip genesis block
            if "encrypted_payload" in block.data:
                try:
                    # Decrypt the data
                    decrypted_data = self.encryption.decrypt_data(block.data["encrypted_payload"])
                    # Store it in the blockchain sensor bucket
                    self.store_blockchain_sensor_data_influx(block, decrypted_data)
                except Exception as e:
                    print(f"Error rebuilding data for block {block.index}: {e}")
        print("Blockchain sensor data rebuild complete!")

    def start(self):
        """Start the data handler"""
        try:
            print("Attempting to connect to MQTT broker...")
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            print("Starting Thingy52 Data Handler with Blockchain...")
            print(f"Blockchain initialized with {len(self.blockchain.chain)} blocks")
            
            # Start MQTT loop in a separate thread for testing
            self.mqtt_client.loop_start()
            
            return self.mqtt_client
            
        except Exception as e:
            print(f"Failed to connect to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            print(f"Error: {e}")
            print("Running in offline mode - blockchain and InfluxDB functionality will still work")
            print("To test with MQTT, ensure broker is running or update MQTT_BROKER address")
            return None

    def simulate_data_flow(self):
        """Simulate Thingy52 data for testing"""
        while True:
            try:
                # Simulate 4 different sensors
                multiple_data = []
                for i in range(4):
                    raw_data = self.nanopb_handler.simulate_thingy52_data()
                    decoded_data = self.nanopb_handler.decode_thingy52_data(raw_data)
                    if decoded_data:
                        decoded_data['sensor_id'] = i
                        multiple_data.append(decoded_data)
                
                if multiple_data:
                    # Process as multiple sensor data
                    payload = {"sensors": multiple_data}
                    self.process_multiple_sensor_data(payload)
                
                time.sleep(10)  # Send data every 10 seconds
            except KeyboardInterrupt:
                print("Stopping simulation...")
                break

    def simulate_single_sensor_data_flow(self):
        """Simulate single Thingy52 sensor data for testing"""
        while True:
            try:
                # Simulate single sensor data
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
        if mqtt_client:
            # For testing: simulate data flow with MQTT
            print("Starting 4-sensor data simulation with MQTT (Press Ctrl+C to stop)...")
            print("This will simulate 4 separate temperature sensors")
            handler.simulate_data_flow()
        else:
            # Offline mode - just test blockchain and InfluxDB
            print("Running in offline mode - testing blockchain functionality...")
            print("Simulating 4 sensors with 5 readings each...")
            for reading in range(5):
                multiple_data = []
                for sensor_id in range(4):
                    raw_data = handler.nanopb_handler.simulate_thingy52_data()
                    decoded_data = handler.nanopb_handler.decode_thingy52_data(raw_data)
                    if decoded_data:
                        decoded_data['sensor_id'] = sensor_id
                        multiple_data.append(decoded_data)
                
                if multiple_data:
                    payload = {"sensors": multiple_data}
                    handler.process_multiple_sensor_data(payload)
                    
                time.sleep(2)
                
            print(f"Blockchain now has {len(handler.blockchain.chain)} blocks")
            
            # Show blockchain summary
            summary = handler.get_blockchain_summary()
            print(f"Blockchain Summary: {json.dumps(summary, indent=2)}")
            
            print("Offline test complete!")
            
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        handler.influx_client.close()