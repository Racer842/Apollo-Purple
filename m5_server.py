#!/usr/bin/env python3
"""
TCP Server for Mac to communicate with M5 Core2
Exchanges periodic hello messages with Core2
"""

import socket
import threading
import time
from datetime import datetime

class M5Core2Server:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.socket = None
        self.client_socket = None
        self.client_address = None
        self.running = False
        self.message_count = 0
        self.start_time = time.time()
        
    def start_server(self):
        """Start the TCP server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(1)
            self.running = True
            
            print(f"Server listening on {self.host}:{self.port}")
            print("Waiting for M5 Core2 connection...")
            
            # Accept connection from M5 Core2
            self.client_socket, self.client_address = self.socket.accept()
            print(f"Connected to M5 Core2 at {self.client_address}")
            print("Starting periodic communication...")
            print("-" * 50)
            
            # Start listening thread for incoming messages
            listen_thread = threading.Thread(target=self.listen_for_messages)
            listen_thread.daemon = True
            listen_thread.start()
            
            # Start periodic message sending thread
            send_thread = threading.Thread(target=self.send_periodic_messages)
            send_thread.daemon = True
            send_thread.start()
            
            return True
            
        except Exception as e:
            print(f"Error starting server: {e}")
            return False
    
    def listen_for_messages(self):
        """Listen for messages from M5 Core2"""
        while self.running and self.client_socket:
            try:
                data = self.client_socket.recv(1024)
                if data:
                    message = data.decode('utf-8').strip()
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    print(f"[{timestamp}] 📱 M5 Core2: {message}")
                else:
                    print("M5 Core2 disconnected")
                    break
            except Exception as e:
                if self.running:  # Only print error if we're still supposed to be running
                    print(f"Error receiving message: {e}")
                break
    
    def send_message(self, message):
        """Send message to M5 Core2"""
        if self.client_socket:
            try:
                self.client_socket.send(f"{message}\n".encode('utf-8'))
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"[{timestamp}] 💻 Sent to M5 Core2: {message}")
                return True
            except Exception as e:
                if self.running:  # Only print error if we're still supposed to be running
                    print(f"Error sending message: {e}")
                return False
        else:
            print("No M5 Core2 connected")
            return False
    
    def send_periodic_messages(self):
        """Send periodic hello messages to M5 Core2"""
        message_interval = 7  # Send every 7 seconds (offset from M5's 5 seconds)
        
        while self.running and self.client_socket:
            try:
                # Calculate uptime
                uptime = int(time.time() - self.start_time)
                
                # Create message
                self.message_count += 1
                message = f"Hello from Mac Server! Message #{self.message_count}, Uptime: {uptime} sec"
                
                # Send message
                self.send_message(message)
                
                # Wait for next interval
                time.sleep(message_interval)
                
            except Exception as e:
                if self.running:
                    print(f"Error in periodic message sending: {e}")
                break
    
    def print_status(self):
        """Print periodic status information"""
        while self.running:
            time.sleep(30)  # Print status every 30 seconds
            if self.running and self.client_socket:
                uptime = int(time.time() - self.start_time)
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"[{timestamp}] 📊 Status: Connected, Uptime: {uptime}s, Messages sent: {self.message_count}")
    
    def stop_server(self):
        """Stop the server"""
        print("\nShutting down server...")
        self.running = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
                
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        
        print("Server stopped")

def get_local_ip():
    """Get the local IP address"""
    try:
        # Connect to a remote address to determine local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "127.0.0.1"

def main():
    print("🚀 M5 Core2 Periodic Communication Server")
    print("=" * 50)
    
    local_ip = get_local_ip()
    print(f"📍 Your Mac's IP address: {local_ip}")
    print(f"🔧 Configure your M5 Core2 to connect to: {local_ip}:8080")
    print(f"⏱️  M5 Core2 sends messages every 5 seconds")
    print(f"⏱️  Mac server sends messages every 7 seconds")
    print()
    
    server = M5Core2Server()
    
    try:
        if server.start_server():
            # Start status printing thread
            status_thread = threading.Thread(target=server.print_status)
            status_thread.daemon = True
            status_thread.start()
            
            print("🎯 Communication active! Press Ctrl+C to stop.")
            print("=" * 50)
            
            # Keep the main thread alive
            while server.running:
                time.sleep(1)
        else:
            print("❌ Failed to start server")
    except KeyboardInterrupt:
        print("\n⏹️  Received Ctrl+C, shutting down...")
    finally:
        server.stop_server()

if __name__ == "__main__":
    main()