#!/bin/bash
#
# Test script for ADIN2111-MK single interface driver
# Author: Murray Kopit <murr2k@gmail.com>
# 
# This script validates basic functionality and network operations
#

set -e

INTERFACE="eth0"
TEST_IP="172.16.1.100"
TEST_NETMASK="255.255.255.0"

echo "=== ADIN2111-MK Driver Test Script ==="
echo

# Check if driver is loaded
echo "1. Checking if adin2111-mk driver is loaded..."
if lsmod | grep -q adin2111_mk; then
    echo "   ✓ Driver is loaded"
else
    echo "   ✗ Driver not found. Attempting to load..."
    if modprobe adin2111-mk; then
        echo "   ✓ Driver loaded successfully"
    else
        echo "   ✗ Failed to load driver"
        exit 1
    fi
fi
echo

# Check if interface exists
echo "2. Checking if $INTERFACE interface exists..."
if ip link show $INTERFACE &> /dev/null; then
    echo "   ✓ Interface $INTERFACE found"
else
    echo "   ✗ Interface $INTERFACE not found"
    echo "Available interfaces:"
    ip link show | grep "^[0-9]" | cut -d: -f2 | tr -d ' '
    exit 1
fi
echo

# Show interface information
echo "3. Interface information:"
ip link show $INTERFACE
echo

# Check PHY status
echo "4. Checking PHY status..."
if ethtool $INTERFACE &> /dev/null; then
    echo "   PHY information:"
    ethtool $INTERFACE | grep -E "(Speed|Duplex|Link detected)"
else
    echo "   ⚠  ethtool not available or interface not ready"
fi
echo

# Test interface configuration
echo "5. Testing interface configuration..."
echo "   Bringing interface down..."
ip link set $INTERFACE down

echo "   Setting IP address: $TEST_IP/$TEST_NETMASK"
if ip addr add $TEST_IP/$TEST_NETMASK dev $INTERFACE; then
    echo "   ✓ IP address set successfully"
else
    echo "   ✗ Failed to set IP address"
    exit 1
fi

echo "   Bringing interface up..."
if ip link set $INTERFACE up; then
    echo "   ✓ Interface brought up successfully"
else
    echo "   ✗ Failed to bring interface up"
    exit 1
fi
echo

# Verify interface status
echo "6. Verifying interface status..."
sleep 2  # Allow time for interface to come up

if ip addr show $INTERFACE | grep -q "$TEST_IP"; then
    echo "   ✓ IP address configured correctly"
    ip addr show $INTERFACE | grep "inet "
else
    echo "   ✗ IP address not found on interface"
fi
echo

# Test basic connectivity (ping local interface)
echo "7. Testing local connectivity..."
if ping -c 3 -W 1 $TEST_IP > /dev/null 2>&1; then
    echo "   ✓ Local ping successful"
else
    echo "   ⚠  Local ping failed (this may be normal depending on setup)"
fi
echo

# Show interface statistics
echo "8. Interface statistics:"
cat /sys/class/net/$INTERFACE/statistics/rx_packets 2>/dev/null && echo " RX packets" || echo "RX packets: unavailable"
cat /sys/class/net/$INTERFACE/statistics/tx_packets 2>/dev/null && echo " TX packets" || echo "TX packets: unavailable"
echo

# Show driver-specific information
echo "9. Driver information:"
if [ -d "/sys/class/net/$INTERFACE/device" ]; then
    echo "   Driver: $(readlink /sys/class/net/$INTERFACE/device/driver 2>/dev/null | xargs basename || echo 'unknown')"
    echo "   Device: $(cat /sys/class/net/$INTERFACE/device/modalias 2>/dev/null || echo 'unknown')"
else
    echo "   Device information not available"
fi
echo

# Test ifconfig compatibility (if available)
echo "10. Testing ifconfig compatibility..."
if command -v ifconfig > /dev/null; then
    echo "    Using ifconfig to show interface:"
    ifconfig $INTERFACE | head -5
    
    echo "    Testing ifconfig configuration:"
    if ifconfig $INTERFACE 172.16.1.101 netmask 255.255.255.0 up; then
        echo "    ✓ ifconfig configuration successful"
        ifconfig $INTERFACE | grep "inet " || echo "    No inet address found"
    else
        echo "    ✗ ifconfig configuration failed"
    fi
else
    echo "    ifconfig not available, using ip commands only"
fi
echo

echo "=== Test Complete ==="
echo "If all tests passed, the ADIN2111-MK driver is working correctly."
echo "The interface should now be available as $INTERFACE and configurable with standard tools."