# User Manual Overview

!!! warning "Important"

    Before using the robot with Flexiv TDK, you must carefully read through all documents shipped with the robot as well as this manual, and strictly follow all safety instructions detailed in these documents.

This manual guides you through installing, configuring, and using the Flexiv TDK (Teleoperation Development Kit) to build teleoperation applications over LAN or WAN.

## Audience
- Application developers integrating teleoperation features
- Robotics engineers deploying Flexiv systems
- Researchers building custom robot-to-robot tele-operation workflows

## Prerequisites

### Robot
- At least two Enlight series robots for TDK v2.x (Rizon series robots with FT sensor configured for TDK v1.x)

### Network Devices
- Network devices (e.g., Ethernet switch/router) with sufficient bandwidth and latency
- CAT 6 or CAT 7 Ethernet cables
### User PC
- Ubuntu 22.04+ (x86_64 or aarch64)
- C++ toolchain: GCC ≥ 9.4, CMake ≥ 3.16.3
- Python 3.10/3.12/3.14
- Network access to the robot and Internet (for WAN)

## Where to Start
- First to finish the [System Setup](./system-setup.md)
- New to the SDK? Start with [Quick Start (Python)](quick-start-python.md) or [Quick Start (C++)](quick-start-cpp.md).
- Need hard real-time performance? See [Real-Time Kernel](real-time-kernel.md).
- Deploying over WAN? See [Time Sync (WAN)](time-sync.md).
- Looking for API doc? See [API doc](../api/index.md).
