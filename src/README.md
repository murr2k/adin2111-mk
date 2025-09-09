# ADIN2111-MK Single Interface Ethernet Driver

**Author:** Murray Kopit <murr2k@gmail.com>

A simplified Linux kernel driver for the Analog Devices ADIN2111 2-Port Ethernet Switch, configured to operate as a single Ethernet interface (eth0) without switching capabilities.

## Overview

This driver is derived from the original ADIN2111 dual-port switch driver but has been extensively modified to:

- **Remove switching/bridge functionality**: Eliminates all switchdev and bridge-related code
- **Single interface operation**: Registers only one network interface (eth0)
- **Simplified configuration**: Direct ethernet operations without dual-port complexity
- **Maintain hardware compatibility**: Preserves all SPI communication and PHY management
- **Standard networking**: Full compatibility with ifconfig, ip, and standard Linux networking tools

## Files

```
src/
├── adin2111-mk.c              # Main driver source code
├── Kconfig                    # Kernel configuration
├── Makefile                   # Build configuration
├── adin2111-mk-overlay.dts    # Device tree overlay example
├── test-adin2111-mk.sh       # Test script for validation
└── README.md                  # This file
```

## Key Features

### ✅ Implemented Features
- Single Ethernet interface (eth0) registration
- Full SPI communication with ADIN2111 hardware
- MDIO/PHY management for 10BASE-T1L
- Standard network device operations
- Interrupt handling for RX/TX operations
- Frame transmission and reception
- MAC address configuration
- Link status monitoring
- Network statistics
- ethtool support
- Compatible with `ifconfig eth0 172.16.1.100 up`

### ❌ Removed Features (from original)
- Dual-port switching logic
- Bridge/switchdev integration
- Spanning Tree Protocol (STP) support
- Forwarding Database (FDB) management
- Inter-port forwarding
- VLAN switching capabilities

## Hardware Requirements

- **ADIN2111 chip**: 2-Port 10BASE-T1L Ethernet Switch MAC-PHY
- **SPI interface**: For communication with the host processor
- **GPIO pins**: For reset and interrupt (optional but recommended)
- **10BASE-T1L PHY**: Integrated in ADIN2111

## Software Requirements

- **Linux Kernel**: 4.14+ (tested on 6.6.48-stm32mp)
- **SPI subsystem**: Kernel SPI support
- **Network stack**: Standard Linux networking
- **Device tree**: For hardware description

## Installation

### 1. Copy Files to Kernel Tree

```bash
# Copy to appropriate kernel source directory
cp src/adin2111-mk.c drivers/net/ethernet/adi/
cp src/Kconfig drivers/net/ethernet/adi/Kconfig.adin2111mk
cp src/Makefile drivers/net/ethernet/adi/Makefile.adin2111mk
```

### 2. Integrate with Kernel Build System

Add to `drivers/net/ethernet/adi/Kconfig`:
```kconfig
source "drivers/net/ethernet/adi/Kconfig.adin2111mk"
```

Add to `drivers/net/ethernet/adi/Makefile`:
```makefile
include drivers/net/ethernet/adi/Makefile.adin2111mk
```

### 3. Configure Kernel

```bash
make menuconfig
# Navigate to: Device Drivers -> Network device support -> Ethernet driver support
# Enable: Analog Devices ADIN2111 Single Interface MAC-PHY driver
```

### 4. Build and Install

```bash
make -j$(nproc)
make modules_install
make install
```

## Device Tree Configuration

### Basic Configuration

```dts
&spi0 {
    adin2111_mk: adin2111-mk@0 {
        compatible = "adi,adin2111-mk";
        reg = <0>;
        spi-max-frequency = <25000000>;
        interrupt-parent = <&gpio>;
        interrupts = <25 2>;  /* GPIO 25, falling edge */
        reset-gpios = <&gpio 24 1>;  /* GPIO 24, active high */
        adi,spi-crc;  /* Enable SPI CRC if supported */
    };
};
```

### Using Device Tree Overlay

```bash
# Compile overlay
dtc -@ -I dts -O dtb -o adin2111-mk-overlay.dtbo src/adin2111-mk-overlay.dts

# Apply overlay (Raspberry Pi example)
sudo cp adin2111-mk-overlay.dtbo /boot/overlays/
echo "dtoverlay=adin2111-mk-overlay" | sudo tee -a /boot/config.txt
sudo reboot
```

## Usage

### Basic Network Configuration

```bash
# Bring interface up with IP address
sudo ifconfig eth0 172.16.1.100 netmask 255.255.255.0 up

# Or using ip command
sudo ip addr add 172.16.1.100/24 dev eth0
sudo ip link set eth0 up

# Check interface status
ifconfig eth0
ip addr show eth0

# Test connectivity
ping 172.16.1.100
```

### Driver Testing

```bash
# Run comprehensive test suite
sudo ./src/test-adin2111-mk.sh

# Check driver loading
lsmod | grep adin2111_mk
dmesg | grep adin2111

# Monitor network activity
sudo tcpdump -i eth0
```

### ethtool Operations

```bash
# Check link status
ethtool eth0

# View interface statistics
ethtool -S eth0

# Check driver information
ethtool -i eth0
```

## Troubleshooting

### Common Issues

1. **Driver not loading**
   ```bash
   # Check kernel messages
   dmesg | grep adin2111
   
   # Verify SPI device
   ls /sys/bus/spi/devices/
   
   # Check device tree
   cat /proc/device-tree/soc/spi*/adin2111*/compatible
   ```

2. **Interface not appearing**
   ```bash
   # Check if driver claims device
   lsmod | grep adin2111_mk
   
   # Verify hardware detection
   cat /sys/class/net/*/device/driver
   ```

3. **SPI communication errors**
   ```bash
   # Check SPI settings
   cat /sys/bus/spi/devices/spi0.0/modalias
   
   # Verify reset and interrupt GPIOs
   gpioinfo | grep -E "(24|25)"
   ```

4. **PHY not detected**
   ```bash
   # Check MDIO bus
   ls /sys/bus/mdio_bus/devices/
   
   # Verify PHY ID
   cat /sys/bus/mdio_bus/devices/*/phy_id
   ```

## Performance Optimization

### Single Interface Benefits
- **Reduced memory usage**: No dual-port structures
- **Lower CPU overhead**: No switching logic
- **Simplified packet path**: Direct netdev operations
- **Faster initialization**: Single PHY setup

### Recommended Settings
```bash
# Optimize for low latency
echo 0 > /proc/sys/net/core/netdev_budget_usecs

# Increase receive buffer
echo 'net.core.rmem_max = 16777216' >> /etc/sysctl.conf

# Enable hardware features
ethtool -K eth0 rx-checksumming on
```

## Development Notes

### Code Architecture
- **Simplified state machine**: Single interface state tracking
- **Direct PHY access**: No port abstraction layer
- **Streamlined interrupts**: Single RX/TX path
- **Optimized memory layout**: Reduced structure sizes

### Key Differences from Original
- Removed `adin1110_port_priv` abstraction
- Eliminated switchdev event handling
- Simplified MAC address management
- Direct network device registration
- Single PHY instance management

## License

This driver is licensed under the same terms as the original ADIN2111 driver:
- **GPL-2.0 OR BSD-2-Clause**

## Contributing

When contributing to this driver:
1. Maintain compatibility with standard Linux networking APIs
2. Preserve all hardware communication protocols
3. Follow kernel coding standards
4. Test with the provided test script
5. Ensure no regressions in basic ethernet functionality

## Support

For issues specific to this single-interface implementation, please check:
1. Kernel messages: `dmesg | grep adin2111`
2. Interface status: `ip link show eth0`
3. Driver status: `lsmod | grep adin2111_mk`
4. Hardware detection: Run test script with debug enabled

For ADIN2111 hardware issues, refer to the official Analog Devices documentation and datasheet.