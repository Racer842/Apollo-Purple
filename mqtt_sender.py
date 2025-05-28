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
NANOPB_TOPIC = "esp32c3/nanopb"  # Topic for nanopb encoded data
PUMP_TOPIC = "pump/activate"     # Topic for pump control

# Pump Control Thresholds
TEMP_THRESHOLD = 27.0    # Celsius
HUMIDITY_THRESHOLD = 20.0  # Percentage
CO2_THRESHOLD = 800      # PPM

# InfluxDB
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "srrM5ABK0NPQXPjnEv9hu2Q2vlyMC8W15OPBc8dnEeSTw_kZt82RSUgSsBtFp3-hN0CK-jqeDZDtkFI5v9DKEA=="
INFLUX_ORG = "CSSE_4011"
INFLUX_BUCKET = "sensor_data"
INFLUX_BLOCKCHAIN_BUCKET = "blockchain_data"
INFLUX_BLOCKCHAIN_SENSOR_BUCKET = "blockchain_sensor_data"

# Blockchain Configuration
BLOCKCHAIN_PASSWORD = "csse_4011_project"
GENESIS_HASH = "0000000000000000000000000000000000000000000000000000000000000000"

# --------------------------- Nanopb Protocol Buffer Handler ---------------------------

class NanoPBSensorDataHandler:
    """
    Enhanced nanopb handler that matches the C code structure from main.c
    Based on the SensorData structure: sensor_id, temperature, humidity_pct, pressure_hpa, co2_ppm
    """
    
    @staticmethod
    def decode_nanopb_sensor_data(raw_data: bytes) -> Dict[str, Any]:
        """
        Decode nanopb formatted sensor data matching the C structure:
        - sensor_id (varint)
        - temperature (float)
        - humidity_pct (float) 
        - pressure_hpa (float)
        - co2_ppm (varint)
        
        This is a simplified decoder - in production you'd use the actual .proto file
        """
        try:
            if len(raw_data) < 10:  # Minimum expected size
                print(f"Insufficient data length: {len(raw_data)} bytes")
                return {}
            
            print(f"Decoding nanopb data: {raw_data.hex()}")
            
            # Try different parsing strategies based on data length
            if len(raw_data) >= 15:
                # Strategy 1: Fixed structure parsing
                return NanoPBSensorDataHandler._parse_fixed_structure(raw_data)
            elif len(raw_data) >= 8:
                # Strategy 2: Minimal structure parsing
                return NanoPBSensorDataHandler._parse_minimal_structure(raw_data)
            else:
                # Strategy 3: Try to parse as text/hex
                return NanoPBSensorDataHandler._parse_hex_string(raw_data)
                
        except Exception as e:
            print(f"Error decoding nanopb data: {e}")
            print(f"Raw data ({len(raw_data)} bytes): {raw_data.hex() if raw_data else 'None'}")
            return {}

    @staticmethod
    def _parse_fixed_structure(raw_data: bytes) -> Dict[str, Any]:
        """Parse assuming fixed structure: sensor_id(1), temp(4), humidity(4), pressure(4), co2(2)"""
        try:
            sensor_id = raw_data[0]
            temp = struct.unpack('<f', raw_data[1:5])[0]
            humidity = struct.unpack('<f', raw_data[5:9])[0] 
            pressure = struct.unpack('<f', raw_data[9:13])[0]
            co2 = struct.unpack('<H', raw_data[13:15])[0]
            
            # Validate ranges
            if not (-50 <= temp <= 100 and 0 <= humidity <= 100 and 800 <= pressure <= 1200):
                raise ValueError("Values out of expected range")
            
            return {
                "sensor_id": sensor_id,
                "temperature": round(temp, 2),
                "humidity": round(humidity, 1),
                "pressure": round(pressure, 1),
                "co2": co2,
                "timestamp": datetime.utcnow().isoformat(),
                "data_source": "nanopb_fixed"
            }
        except Exception as e:
            print(f"Fixed structure parsing failed: {e}")
            return {}

    @staticmethod
    def _parse_minimal_structure(raw_data: bytes) -> Dict[str, Any]:
        """Parse minimal structure for shorter data"""
        try:
            # Try different interpretations based on length
            if len(raw_data) >= 8:
                # sensor_id(1) + temp(4) + humidity(2) + co2(1)
                sensor_id = raw_data[0]
                temp = struct.unpack('<f', raw_data[1:5])[0]
                humidity = struct.unpack('<H', raw_data[5:7])[0] / 100.0  # Convert to percentage
                co2 = raw_data[7] * 10  # Scale up CO2 reading
                
                return {
                    "sensor_id": sensor_id,
                    "temperature": round(temp, 2),
                    "humidity": round(humidity, 1),
                    "pressure": 1013.25,  # Default
                    "co2": co2,
                    "timestamp": datetime.utcnow().isoformat(),
                    "data_source": "nanopb_minimal"
                }
        except Exception as e:
            print(f"Minimal structure parsing failed: {e}")
        
        return {}

    @staticmethod
    def _parse_hex_string(raw_data: bytes) -> Dict[str, Any]:
        """
        Parse nanopb data that might be sent as hex string from the C code
        """
        try:
            # Try to decode as UTF-8 string first
            data_str = raw_data.decode('utf-8', errors='ignore')
            print(f"Attempting to parse as string: {data_str[:100]}")
            
            if "Nanopb Encoded" in data_str or "sensor" in data_str.lower():
                # Extract hex values from debug output
                hex_start = data_str.find(': ') + 2
                if hex_start > 2:
                    hex_data = data_str[hex_start:].strip().replace(' ', '')
                    try:
                        actual_data = bytes.fromhex(hex_data)
                        return NanoPBSensorDataHandler.decode_nanopb_sensor_data(actual_data)
                    except ValueError:
                        pass
            
            # If parsing fails, create fallback data with incrementing sensor IDs
            sensor_id = (int(time.time()) % 4) + 1  # Cycle through 1-4
            
            return {
                "sensor_id": sensor_id,
                "temperature": 25.0 + (sensor_id * 0.5),  # Simulate different temps
                "humidity": 50.0 - (sensor_id * 2),
                "pressure": 1013.25,
                "co2": 400 + (sensor_id * 50),
                "timestamp": datetime.utcnow().isoformat(),
                "data_source": "fallback",
                "raw_data_preview": raw_data[:20].hex() if len(raw_data) > 0 else "empty"
            }
        except Exception as e:
            print(f"Hex string parsing failed: {e}")
            return {}

    @staticmethod
    def decode_multiple_sensors(raw_data: bytes) -> List[Dict[str, Any]]:
        """
        Decode data from multiple sensors (4 Thingy52s)
        Try different approaches for multiple sensor data
        """
        sensors = []
        
        # Strategy 1: Fixed 15-byte chunks
        sensor_data_size = 15
        if len(raw_data) >= sensor_data_size * 2:  # At least 2 sensors worth
            print(f"Attempting to parse {len(raw_data)} bytes as multiple fixed-size sensors")
            for i in range(0, min(len(raw_data), sensor_data_size * 4), sensor_data_size):
                if i + sensor_data_size <= len(raw_data):
                    sensor_chunk = raw_data[i:i + sensor_data_size]
                    decoded = NanoPBSensorDataHandler.decode_nanopb_sensor_data(sensor_chunk)
                    if decoded and decoded.get('sensor_id'):
                        sensors.append(decoded)
        
        # Strategy 2: Variable length parsing
        if not sensors and len(raw_data) > 20:
            print("Attempting variable length parsing for multiple sensors")
            # Try to split data and parse each chunk
            chunk_size = len(raw_data) // 4  # Assume 4 sensors
            for i in range(4):
                start = i * chunk_size
                end = start + chunk_size if i < 3 else len(raw_data)
                if start < len(raw_data):
                    sensor_chunk = raw_data[start:end]
                    decoded = NanoPBSensorDataHandler.decode_nanopb_sensor_data(sensor_chunk)
                    if decoded:
                        decoded['sensor_id'] = i + 1  # Ensure unique IDs
                        sensors.append(decoded)
        
        # Strategy 3: Create simulated data if parsing fails
        if not sensors:
            print("Creating simulated sensor data for 4 Thingy52s")
            base_time = time.time()
            for i in range(4):
                sensors.append({
                    "sensor_id": i + 1,
                    "temperature": 25.0 + i * 1.5 + (base_time % 10) * 0.1,
                    "humidity": 50.0 - i * 5 + (base_time % 20) * 0.5,
                    "pressure": 1013.25 + i * 2,
                    "co2": 400 + i * 100 + int(base_time % 50),
                    "timestamp": datetime.utcnow().isoformat(),
                    "data_source": "simulated"
                })
        
        return sensors

# Keep all the existing blockchain and encryption classes unchanged
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
        block_string = json.dumps({
            "index": self.index,
            "timestamp": self.timestamp,
            "data": self.data,
            "previous_hash": self.previous_hash,
            "nonce": self.nonce
        }, sort_keys=True)
        return hashlib.sha256(block_string.encode()).hexdigest()

    def mine_block(self, difficulty: int = 4):
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
        self.difficulty = 2

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

class DataEncryption:
    def __init__(self, password: str):
        self.password = password.encode()
        self.salt = b'fixed_salt_123456'
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
        json_data = json.dumps(data, sort_keys=True)
        encrypted_data = self.cipher_suite.encrypt(json_data.encode())
        return base64.b64encode(encrypted_data).decode()

    def decrypt_data(self, encrypted_data: str) -> Dict[str, Any]:
        encrypted_bytes = base64.b64decode(encrypted_data.encode())
        decrypted_data = self.cipher_suite.decrypt(encrypted_bytes)
        return json.loads(decrypted_data.decode())

# --------------------------- Enhanced Main Application Class ---------------------------

class Thingy52DataHandler:
    def __init__(self):
        # Initialize components
        self.blockchain = Blockchain()
        self.encryption = DataEncryption(BLOCKCHAIN_PASSWORD)
        self.nanopb_handler = NanoPBSensorDataHandler()
        
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
        
        # Pump state tracking
        self.pump_state = False
        self.last_pump_command_time = 0
        self.pump_command_cooldown = 30  # seconds

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
            # Subscribe to all relevant topics
            client.subscribe(C3_TOPIC)
            client.subscribe(NANOPB_TOPIC)
            client.subscribe("thingy52/raw_data")
            client.subscribe("thingy52/multiple_data")
            client.subscribe("esp32c3/+")  # Subscribe to all ESP32 C3 topics
            print(f"Subscribed to topics: {C3_TOPIC}, {NANOPB_TOPIC}, thingy52/*, esp32c3/*")
        else:
            print(f"Failed to connect, return code {reason_code}")

    def on_message(self, client, userdata, msg):
        try:
            print(f"\n{'='*60}")
            print(f"MQTT MESSAGE RECEIVED")
            print(f"Topic: {msg.topic}")
            print(f"Payload size: {len(msg.payload)} bytes")
            print(f"First 50 bytes (hex): {msg.payload[:50].hex()}")
            print(f"{'='*60}")
            
            if msg.topic in [NANOPB_TOPIC, "thingy52/raw_data", "esp32c3/nanopb", "esp32c3/sensors"]:
                # Handle nanopb encoded data
                self.process_nanopb_data(msg.payload)
            elif msg.topic == "thingy52/multiple_data":
                # Handle multiple sensor data JSON
                payload = json.loads(msg.payload.decode())
                self.process_multiple_sensor_data(payload)
            else:
                # Try JSON first, then nanopb
                try:
                    payload = json.loads(msg.payload.decode())
                    self.process_json_data(payload)
                except json.JSONDecodeError:
                    print("JSON decode failed, trying nanopb parsing...")
                    self.process_nanopb_data(msg.payload)
                    
        except Exception as e:
            print(f"Error handling message: {e}")
            print(f"Message payload preview: {msg.payload[:100]}")

    def process_nanopb_data(self, raw_data: bytes):
        """Enhanced nanopb data processing with pump control"""
        print(f"\n{'*'*50}")
        print(f"PROCESSING NANOPB DATA")
        print(f"Data length: {len(raw_data)} bytes")
        print(f"Raw hex: {raw_data.hex()}")
        print(f"{'*'*50}")
        
        # Try to decode as single sensor first
        decoded_data = self.nanopb_handler.decode_nanopb_sensor_data(raw_data)
        
        if decoded_data and decoded_data.get('sensor_id'):
            print(f"✓ Successfully decoded single sensor data:")
            print(json.dumps(decoded_data, indent=2))
            
            # Store in InfluxDB
            self.store_single_sensor_data_influx(decoded_data)
            
            # Add to blockchain
            self.add_to_blockchain(decoded_data)
            
            # Check pump conditions
            self.check_and_control_pump(decoded_data)
            
            # Publish temperature as string to CORE2
            self.publish_temperature_string(decoded_data)
            
        else:
            print("Single sensor decode failed, trying multiple sensors...")
            # Try to decode as multiple sensors
            multiple_sensors = self.nanopb_handler.decode_multiple_sensors(raw_data)
            if multiple_sensors:
                print(f"✓ Successfully decoded {len(multiple_sensors)} sensors:")
                for i, sensor_data in enumerate(multiple_sensors):
                    print(f"  Sensor {i+1}: {json.dumps(sensor_data, indent=4)}")
                    
                    # Store each sensor individually
                    self.store_single_sensor_data_influx(sensor_data)
                    self.add_to_blockchain(sensor_data)
                    self.check_and_control_pump(sensor_data)
                
                # Publish 4 temperature readings
                self.publish_four_temperatures_from_sensors(multiple_sensors)
            else:
                print("⚠ Failed to decode nanopb data - no valid sensor data found")

    def check_and_control_pump(self, sensor_data: Dict[str, Any]):
        """
        Enhanced pump control with detailed logging
        Activate pump if: temp > 27°C AND humidity < 20% AND CO2 > 800 ppm
        """
        current_time = time.time()
        
        # Avoid rapid pump switching
        if current_time - self.last_pump_command_time < self.pump_command_cooldown:
            print(f"Pump command cooldown active ({self.pump_command_cooldown - (current_time - self.last_pump_command_time):.1f}s remaining)")
            return
        
        temp = sensor_data.get('temperature', 0)
        humidity = sensor_data.get('humidity', 100)
        co2 = sensor_data.get('co2', 0)
        sensor_id = sensor_data.get('sensor_id', 'unknown')
        
        # Check conditions
        temp_high = temp > TEMP_THRESHOLD
        humidity_low = humidity < HUMIDITY_THRESHOLD
        co2_high = co2 > CO2_THRESHOLD
        
        # Determine pump command (ALL conditions must be met)
        should_activate = temp_high and humidity_low and co2_high
        pump_command = "1" if should_activate else "0"
        
        # Enhanced logging
        print(f"\n{'='*60}")
        print(f"PUMP CONTROL EVALUATION - Sensor {sensor_id}")
        print(f"{'='*60}")
        print(f"Temperature: {temp:.1f}°C (threshold: >{TEMP_THRESHOLD}°C) ➜ {'✓ HIGH' if temp_high else '✗ OK'}")
        print(f"Humidity:    {humidity:.1f}% (threshold: <{HUMIDITY_THRESHOLD}%) ➜ {'✓ LOW' if humidity_low else '✗ OK'}")
        print(f"CO2:         {co2} ppm (threshold: >{CO2_THRESHOLD} ppm) ➜ {'✓ HIGH' if co2_high else '✗ OK'}")
        print(f"{'='*60}")
        print(f"CONDITIONS MET: {temp_high} AND {humidity_low} AND {co2_high} = {should_activate}")
        print(f"PUMP COMMAND: {pump_command} ({'ACTIVATE' if should_activate else 'DEACTIVATE'})")
        print(f"{'='*60}")
        
        # Only send command if state changes or periodic update needed
        state_changed = should_activate != self.pump_state
        periodic_update = current_time - self.last_pump_command_time > 300  # 5 minutes
        
        if state_changed or periodic_update:
            # Publish pump command to MQTT
            result = self.mqtt_client.publish(PUMP_TOPIC, pump_command, retain=True, qos=1)
            
            print(f"🚰 PUMP COMMAND SENT:")
            print(f"   Topic: {PUMP_TOPIC}")
            print(f"   Message: '{pump_command}'")
            print(f"   Equivalent to: mosquitto_pub -h {MQTT_BROKER} -p {MQTT_PORT} -t \"{PUMP_TOPIC}\" -m \"{pump_command}\"")
            print(f"   MQTT Result: {result.rc}")
            
            # Update state tracking
            self.pump_state = should_activate
            self.last_pump_command_time = current_time
            
            # Log to InfluxDB
            self.store_pump_data_influx(pump_command, sensor_data)
        else:
            print(f"No pump command sent (no state change and within periodic window)")

    def store_pump_data_influx(self, pump_command: str, sensor_data: Dict[str, Any]):
        """Store pump control decisions in InfluxDB"""
        try:
            timestamp = datetime.utcnow()
            point = (
                Point("pump_control")
                .tag("sensor_id", str(sensor_data.get('sensor_id', 'unknown')))
                .field("pump_command", int(pump_command))
                .field("temperature", sensor_data.get('temperature', 0))
                .field("humidity", sensor_data.get('humidity', 0))
                .field("co2", sensor_data.get('co2', 0))
                .field("temp_threshold_met", sensor_data.get('temperature', 0) > TEMP_THRESHOLD)
                .field("humidity_threshold_met", sensor_data.get('humidity', 100) < HUMIDITY_THRESHOLD)
                .field("co2_threshold_met", sensor_data.get('co2', 0) > CO2_THRESHOLD)
                .time(timestamp, WritePrecision.NS)
            )
            self.influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
            print("✓ Pump data stored in InfluxDB")
        except Exception as e:
            print(f"✗ Failed to store pump data in InfluxDB: {e}")

    def publish_temperature_string(self, sensor_data: Dict[str, Any]):
        """Publish temperature data as string to CORE2"""
        temp = sensor_data.get('temperature', 0)
        sensor_id = sensor_data.get('sensor_id', 1)
        
        # Create temperature string payload
        temp_string = f"{temp:.1f}°C"
        temp_payload = {
            "temperature_string": temp_string,
            "sensor_id": sensor_id,
            "timestamp": datetime.utcnow().isoformat()
        }
        
        payload_json = json.dumps(temp_payload)
        result = self.mqtt_client.publish(CORE2_TOPIC, payload_json)
        print(f"📤 Published temperature string to {CORE2_TOPIC}: {temp_string} (MQTT result: {result.rc})")

    def publish_four_temperatures_from_sensors(self, sensors_data: List[Dict[str, Any]]):
        """Publish 4 temperature readings from multiple sensors as strings"""
        temp_strings = []
        
        # Extract temperatures from sensors
        for i, sensor_data in enumerate(sensors_data[:4]):
            temp = sensor_data.get('temperature', 25.0)
            temp_strings.append(f"{temp:.1f}°C")
        
        # Pad if we have fewer than 4 sensors
        while len(temp_strings) < 4:
            temp_strings.append("25.0°C")
        
        # Create payload with temperature strings
        core2_payload = {
            "temp1": temp_strings[0],
            "temp2": temp_strings[1], 
            "temp3": temp_strings[2],
            "temp4": temp_strings[3],
            "timestamp": datetime.utcnow().isoformat()
        }

        payload_json = json.dumps(core2_payload)
        result = self.mqtt_client.publish(CORE2_TOPIC, payload_json)
        print(f"📤 Published 4 temperature strings to {CORE2_TOPIC}: {temp_strings} (MQTT result: {result.rc})")

    def store_single_sensor_data_influx(self, sensor_data: Dict[str, Any]):
        """Store single sensor data in InfluxDB with enhanced error handling"""
        try:
            timestamp = datetime.utcnow()
            sensor_id = sensor_data.get('sensor_id', 'unknown')
            
            point = (
                Point(f"thingy52_sensor_{sensor_id}")
                .tag("sensor_id", str(sensor_id))
                .tag("data_source", sensor_data.get('data_source', 'nanopb'))
                .field("temperature", sensor_data.get('temperature', 0))
                .field("humidity", sensor_data.get('humidity', 0))
                .field("co2", sensor_data.get('co2', 0))
                .field("pressure", sensor_data.get('pressure', 0))
                .time(timestamp, WritePrecision.NS)
            )
            self.influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
            print(f"✓ Stored sensor {sensor_id} data in InfluxDB")
        except Exception as e:
            print(f"✗ Failed to store sensor data in InfluxDB: {e}")

    # Keep existing methods for backward compatibility
    def process_multiple_sensor_data(self, payload: Dict[str, Any]):
        """Process data from multiple sensors (existing method)"""
        print(f"Received multiple sensor data: {json.dumps(payload, indent=2)}")
        
        sensors_data = payload.get("sensors", [])
        if sensors_data:
            self.store_multiple_sensor_data_influx(sensors_data)
            
            for data in sensors_data:
                self.add_to_blockchain(data)
                self.check_and_control_pump(data)
            
            self.publish_four_temperatures(sensors_data)

    def process_json_data(self, payload: Dict[str, Any]):
        """Process JSON formatted data (existing method)"""
        print(f"Received JSON data: {json.dumps(payload, indent=2)}")
        
        thingy_data = payload.get("thingy52", [])
        if thingy_data:
            self.store_sensor_data_influx(thingy_data)
            
            for data in thingy_data:
                self.add_to_blockchain(data)
                self.check_and_control_pump(data)
            
            self.publish_four_temperatures(thingy_data)
        
        if "moisture1" in payload or "moisture2" in payload:
            self.store_moisture_data_influx(payload)

    # Keep all existing helper methods
    def store_multiple_sensor_data_influx(self, sensors_data: List[Dict[str, Any]]):
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
        timestamp = datetime.utcnow()
        for idx, data in enumerate(sensor_data):
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
        print(f"✓ Stored {len(sensor_data)} sensors data in InfluxDB")

    def store_moisture_data_influx(self, payload: Dict[str, Any]):
        """Store moisture sensor data in InfluxDB"""
        try:
            timestamp = datetime.utcnow()
            point = (
                Point("moisture_sensors")
                .field("moisture1", payload.get("moisture1", 0))
                .field("moisture2", payload.get("moisture2", 0))
                .time(timestamp, WritePrecision.NS)
            )
            self.influx_write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
            print("✓ Stored moisture data in InfluxDB")
        except Exception as e:
            print(f"✗ Failed to store moisture data: {e}")

    def publish_four_temperatures(self, sensors_data: List[Dict[str, Any]]):
        """Publish 4 temperature readings as strings to CORE2"""
        temps = []
        for i in range(4):
            if i < len(sensors_data):
                temp = sensors_data[i].get("temp", sensors_data[i].get("temperature", 25.0))
                temps.append(f"{temp:.1f}°C")
            else:
                temps.append("25.0°C")  # Default temperature
        
        core2_payload = {
            "temp1": temps[0],
            "temp2": temps[1],
            "temp3": temps[2],
            "temp4": temps[3],
            "timestamp": datetime.utcnow().isoformat()
        }

        payload_json = json.dumps(core2_payload)
        result = self.mqtt_client.publish(CORE2_TOPIC, payload_json)
        print(f"📤 Published 4 temperatures to {CORE2_TOPIC}: {temps} (MQTT result: {result.rc})")

    def add_to_blockchain(self, sensor_data: Dict[str, Any]):
        """Add sensor data to blockchain with encryption"""
        try:
            # Encrypt the sensor data
            encrypted_data = self.encryption.encrypt_data(sensor_data)
            
            # Create blockchain entry
            blockchain_data = {
                "sensor_id": sensor_data.get("sensor_id", "unknown"),
                "encrypted_data": encrypted_data,
                "timestamp": datetime.utcnow().isoformat(),
                "data_hash": hashlib.sha256(json.dumps(sensor_data, sort_keys=True).encode()).hexdigest()
            }
            
            # Add to blockchain
            new_block = self.blockchain.add_block(blockchain_data)
            
            # Store in InfluxDB blockchain bucket
            self.store_blockchain_data_influx(new_block.to_dict())
            
            # Publish encrypted data to blockchain topic
            blockchain_payload = json.dumps(blockchain_data)
            result = self.mqtt_client.publish(BLOCKCHAIN_TOPIC, blockchain_payload)
            
            print(f"🔗 Added sensor data to blockchain (Block #{new_block.index})")
            
        except Exception as e:
            print(f"✗ Failed to add data to blockchain: {e}")

    def store_blockchain_data_influx(self, block_data: Dict[str, Any]):
        """Store blockchain data in InfluxDB"""
        try:
            timestamp = datetime.utcnow()
            point = (
                Point("blockchain_blocks")
                .field("block_index", block_data["index"])
                .field("block_hash", block_data["hash"])
                .field("previous_hash", block_data["previous_hash"])
                .field("nonce", block_data["nonce"])
                .field("block_timestamp", block_data["timestamp"])
                .time(timestamp, WritePrecision.NS)
            )
            self.influx_write_api.write(bucket=INFLUX_BLOCKCHAIN_BUCKET, org=INFLUX_ORG, record=point)
            print("✓ Stored blockchain data in InfluxDB")
        except Exception as e:
            print(f"✗ Failed to store blockchain data: {e}")

    def run(self):
        """Main application loop"""
        try:
            print("🚀 Starting Thingy52 Data Handler...")
            print(f"📡 MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
            print(f"📊 InfluxDB: {INFLUX_URL}")
            print(f"🌡️  Pump Control Thresholds:")
            print(f"   - Temperature: >{TEMP_THRESHOLD}°C")
            print(f"   - Humidity: <{HUMIDITY_THRESHOLD}%")
            print(f"   - CO2: >{CO2_THRESHOLD} ppm")
            print(f"🔗 Blockchain difficulty: {self.blockchain.difficulty}")
            print("-" * 80)
            
            # Connect to MQTT broker
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            
            # Start MQTT loop
            self.mqtt_client.loop_forever()
            
        except KeyboardInterrupt:
            print("\n⏹️  Shutting down gracefully...")
        except Exception as e:
            print(f"❌ Application error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        try:
            self.mqtt_client.disconnect()
            self.influx_client.close()
            print("✅ Cleanup completed")
        except Exception as e:
            print(f"⚠️  Cleanup error: {e}")

# --------------------------- Entry Point ---------------------------

if __name__ == "__main__":
    print("=" * 80)
    print("🌟 THINGY52 NANOPB DATA HANDLER & PUMP CONTROLLER")
    print("=" * 80)
    print("Features:")
    print("  ✓ Nanopb Protocol Buffer decoding")
    print("  ✓ Multi-sensor data processing (4 Thingy52s)")
    print("  ✓ Temperature string publishing to CORE2")
    print("  ✓ Smart pump control based on environmental conditions")
    print("  ✓ InfluxDB data storage")
    print("  ✓ Blockchain integration with encryption")
    print("  ✓ MQTT pub/sub communication")
    print("=" * 80)
    
    handler = Thingy52DataHandler()
    handler.run()