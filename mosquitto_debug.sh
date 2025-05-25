#!/bin/bash

echo "=== Mosquitto MQTT Broker Debug Script ==="
echo

# Check if Mosquitto is installed
echo "1. Checking Mosquitto installation..."
if command -v mosquitto &> /dev/null; then
    echo "✓ Mosquitto is installed"
    mosquitto -h | head -3
else
    echo "✗ Mosquitto not found in PATH"
fi
echo

# Check if Mosquitto is running
echo "2. Checking if Mosquitto is running..."
if pgrep mosquitto > /dev/null; then
    echo "✓ Mosquitto process is running"
    ps aux | grep mosquitto | grep -v grep
else
    echo "✗ Mosquitto is not running"
    echo "Start it with: brew services start mosquitto"
    echo "Or manually: mosquitto -c /path/to/mosquitto.conf"
fi
echo

# Check listening ports
echo "3. Checking listening ports..."
echo "Ports used by Mosquitto:"
sudo lsof -i -P | grep mosquitto || echo "No Mosquitto ports found"
echo
echo "All processes listening on port 1883:"
sudo lsof -i :1883 || echo "Nothing listening on port 1883"
echo

# Find config file
echo "4. Finding Mosquitto configuration..."
CONFIG_LOCATIONS=(
    "/usr/local/etc/mosquitto/mosquitto.conf"
    "/opt/homebrew/etc/mosquitto/mosquitto.conf"
    "/etc/mosquitto/mosquitto.conf"
    "$(brew --prefix)/etc/mosquitto/mosquitto.conf"
)

for config in "${CONFIG_LOCATIONS[@]}"; do
    if [[ -f "$config" ]]; then
        echo "✓ Found config at: $config"
        echo "Key settings:"
        grep -E "^(port|listener|allow_anonymous|bind_address)" "$config" 2>/dev/null || echo "No specific port/listener settings found"
        FOUND_CONFIG="$config"
        break
    fi
done

if [[ -z "$FOUND_CONFIG" ]]; then
    echo "✗ No config file found in common locations"
fi
echo

# Check network interfaces and IPs
echo "5. Network configuration..."
echo "Your Mac's IP addresses:"
ifconfig | grep "inet " | grep -v 127.0.0.1 | while read line; do
    echo "  $line"
done
echo

# Test local MQTT connection
echo "6. Testing local MQTT connection..."
if command -v mosquitto_pub &> /dev/null && command -v mosquitto_sub &> /dev/null; then
    echo "Testing localhost connection..."
    
    # Start subscriber in background
    timeout 3 mosquitto_sub -h localhost -t test/debug &
    SUB_PID=$!
    sleep 1
    
    # Publish test message
    mosquitto_pub -h localhost -t test/debug -m "test message $(date)"
    
    # Wait a moment for message
    sleep 1
    kill $SUB_PID 2>/dev/null
    
    if [[ $? -eq 0 ]]; then
        echo "✓ Local MQTT test successful"
    else
        echo "✗ Local MQTT test failed"
    fi
else
    echo "mosquitto_pub/sub not available for testing"
fi
echo

# Check firewall
echo "7. Checking macOS firewall..."
FIREWALL_STATE=$(sudo /usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate 2>/dev/null)
echo "Firewall state: $FIREWALL_STATE"
echo

# Provide recommendations
echo "=== RECOMMENDATIONS ==="
echo
echo "For M5Core2 connection, ensure:"
echo "1. Both devices are on the same network"
echo "2. Mosquitto config has: listener 1883 0.0.0.0"
echo "3. Mosquitto config has: allow_anonymous true"
echo "4. Firewall allows port 1883"
echo "5. Use the correct IP address in your M5Core2 code"
echo
echo "Your current network IPs to try:"
ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print "  " $2}'
echo
echo "Test from M5Core2 network with:"
echo "  mosquitto_pub -h YOUR_MAC_IP -t test -m hello"