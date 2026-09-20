# 用户手册概述

!!! warning "重要"

    在使用 Flexiv TDK 操作机器人之前，必须仔细阅读随机器人附带的全部文档以及本手册，并严格遵守这些文档中的所有安全说明。

本手册将指导你安装、配置和使用 Flexiv TDK（遥操作开发套件），构建基于局域网（LAN）或广域网（WAN）的遥操作应用。

## 适用读者

- 集成遥操作功能的应用开发者
- 部署 Flexiv 系统的机器人工程师
- 构建自定义"机器人对机器人"遥操作工作流的研究人员

## 前提条件

### 机器人
- TDK v2.x 至少需要两台 Enlight 系列机器人（TDK v1.x 为配置有FT力传感器的 Rizon 系列机器人）

### 网络设备
- 带宽和时延满足要求的网络设备（如以太网交换机/路由器）
- CAT 6 或 CAT 7 网线

### 用户计算机
- Ubuntu 22.04+（x86_64 或 aarch64）：C++ 和 Python 3.10/3.12/3.14
- macOS 14+（arm64）：C++ 和 Python 3.10/3.12
- C++ 工具链：GCC ≥ 9.4（Linux）或 Apple Clang ≥ 15（macOS），CMake ≥ 3.16.3
- 可访问机器人网络及互联网（WAN 场景）

## 从这里开始

- 首先完成[系统搭建](./system-setup.md)
- 初次接触 SDK？从[快速上手 (Python)](quick-start-python.md) 或[快速上手 (C++)](quick-start-cpp.md) 开始。
- 需要硬实时性能？参见[实时内核](real-time-kernel.md)。
- 通过 WAN 部署？参见[时间同步 (WAN)](time-sync.md)。
- 查找 API 文档？参见 [API 文档](../api/index.md)。
