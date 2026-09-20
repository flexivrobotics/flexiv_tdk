# User Manual Overview

This manual guides you through installing, configuring, and using the Flexiv TDK (Teleoperation Development Kit) to build teleoperation applications over LAN or WAN.

This documentation source is **TDK v1.6.4**. Version mapping for this release:

| TDK | C++ RDK | Python RDK |
| --- | ------- | ---------- |
| **v1.6.4** (`flexivtdk==1.6.4`) | **flexiv_rdk 1.9.4** (only public C++ dependency) | **flexivrdk==1.9.4** |

The online GitHub Pages site documents TDK v2.x and must not be used with this release. Generate this manual locally (see the repository README).

## Audience
- Application developers integrating teleoperation features
- Robotics engineers deploying Flexiv systems
- Researchers building custom haptic or robot-to-robot workflows

## Prerequisites

### Robot
- At least two Flexiv Rizon robots with FT sensors configured
### Network Devices
- Network devices (e.g., Ethernet switch/router) with sufficient bandwidth and latency
- CAT 6 or CAT 7 Ethernet cables
### User PC
- Ubuntu 22.04+ (x86_64 or aarch64): C++ and Python 3.10/3.12/3.14
- macOS 14+ (arm64): C++ and Python 3.10/3.12
- C++ toolchain: GCC ≥ 9.4 (Linux) or Apple Clang ≥ 15 (macOS), CMake ≥ 3.16.3
- Network access to the robot and (for WAN) reliable time synchronization

## Repository Layout
- `include/`: C++ headers
- `example/`: C++ example apps
- `example_py/`: Python example apps
- `docs/doxygen/`: Doxygen configuration and assets

## Where to Start
- New to the SDK? Start with [Quick Start (Python)](quick-start-python.md) or [Quick Start (C++)](quick-start-cpp.md).
- Need hard real-time performance? See [Real-Time Kernel](real-time-kernel.md).
- Deploying over WAN? See [Time Sync (WAN)](time-sync.md).
- Looking for sample usage? See [Examples](examples.md).
- Third-party licenses for the prebuilt library: see [THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md) in the repository.
