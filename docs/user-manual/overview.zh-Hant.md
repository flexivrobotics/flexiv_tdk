# 使用者手冊概述

!!! warning "重要"

    在使用 Flexiv TDK 操作機器人之前，必須仔細閱讀隨機器人附帶的全部文件以及本手冊，並嚴格遵守這些文件中的所有安全說明。

本手冊將指導你安裝、設定和使用 Flexiv TDK（遙操作開發套件），建構基於區域網路（LAN）或廣域網路（WAN）的遙操作應用。

## 適用讀者

- 整合遙操作功能的應用開發者
- 部署 Flexiv 系統的機器人工程師
- 建構自訂「機器人對機器人」遙操作工作流程的研究人員

## 前提條件

### 機器人
- TDK v2.x 至少需要兩台 Enlight 系列機器人（TDK v1.x 為配置有FT力感測器的 Rizon 系列機器人）

### 網路設備
- 頻寬和延遲滿足要求的網路設備（如乙太網路交換器/路由器）
- CAT 6 或 CAT 7 網線

### 使用者電腦
- Ubuntu 22.04+（x86_64 或 aarch64）：C++ 和 Python 3.10/3.12/3.14
- macOS 14+（arm64）：C++ 和 Python 3.10/3.12
- C++ 工具鏈：GCC ≥ 9.4（Linux）或 Apple Clang ≥ 15（macOS），CMake ≥ 3.16.3
- 可存取機器人網路及網際網路（WAN 場景）

## 從這裡開始

- 首先完成[系統設定](./system-setup.md)
- 初次接觸 SDK？從[快速上手 (Python)](quick-start-python.md) 或[快速上手 (C++)](quick-start-cpp.md) 開始。
- 需要硬即時效能？參見[即時核心](real-time-kernel.md)。
- 透過 WAN 部署？參見[時間同步 (WAN)](time-sync.md)。
- 查找 API 文件？參見 [API 文件](../api/index.md)。
