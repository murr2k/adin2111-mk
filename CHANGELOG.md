# Changelog

All notable changes to the ADIN2111-MK Single Interface Ethernet Driver will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2025-01-09

### Added
- Initial release of ADIN2111-MK single interface Ethernet driver
- Single eth0 interface operation derived from ADIN2111 dual-port driver
- Support for `ifconfig eth0 172.16.1.100 up` simplified configuration
- Complete removal of bridge/switchdev functionality for embedded applications
- Comprehensive test suite for driver validation
- Device tree overlay for hardware configuration
- Kernel configuration and Makefile for build system
- Full documentation and usage examples

### Changed
- Simplified from dual-port switching to single interface operation
- Removed all bridge-related dependencies and switchdev framework
- Optimized memory usage and CPU overhead for single interface
- Streamlined interrupt handling for single RX/TX path

### Removed
- Dual-port switching logic and port abstractions
- Bridge join/leave operations and STP support
- Forwarding Database (FDB) management
- Inter-port forwarding and VLAN switching capabilities
- switchdev infrastructure and event handling

### Technical Details
- Based on original ADIN2111 driver with ~60% code reduction
- Maintains all essential ADIN2111 hardware communication protocols
- Compatible with Ubuntu 6.6.48-stm32mp kernel
- Supports 10BASE-T1L Ethernet standard
- Preserves SPI communication and MDIO/PHY management
- Author: Murray Kopit <murr2k@gmail.com>

[Unreleased]: https://github.com/murr2k/adin2111-mk/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/murr2k/adin2111-mk/releases/tag/v1.0.0