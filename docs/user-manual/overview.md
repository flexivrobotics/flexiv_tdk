# User Manual Overview

This manual guides you through installing, configuring, and using the Flexiv TDK (Teleoperation Development Kit) to build teleoperation applications over LAN or WAN.

## Audience
- Application developers integrating teleoperation features
- Robotics engineers deploying Flexiv systems
- Researchers building custom haptic or robot-to-robot workflows

## Prerequisites
- Ubuntu 22.04+ (x86_64 or aarch64)
- C++ toolchain: GCC ≥ 9.4, CMake ≥ 3.16.3
- Python 3.8/3.10/3.12 (for Python SDK usage)
- Network access to the robot and (for WAN) reliable time synchronization

## Repository Layout
- `include/`: C++ headers
- `example/`: C++ example apps
- `example_py/`: Python example apps
- `doc/`: Doxygen configuration and assets

## Where to Start
- New to the SDK? Start with [Quick Start (Python)](quick-start-python.md) or [Quick Start (C++)](quick-start-cpp.md).
- Need hard real-time performance? See [Real-Time Kernel](real-time-kernel.md).
- Deploying over WAN? See [Time Sync (WAN)](time-sync.md).
- Looking for sample usage? See [Examples](examples.md).
