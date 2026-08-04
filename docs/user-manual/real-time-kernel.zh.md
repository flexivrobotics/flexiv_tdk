# 实时内核选项

并非所有使用场景都需要实时操作系统，请阅读下文判断你是否需要。

## ⚙️ 实时性能的内核选项

Ubuntu 提供多种面向不同工作负载的内核变体：

| 内核类型            | 说明                                       | 典型使用场景               |
| ------------------- | ------------------------------------------ | -------------------------- |
| `generic`           | 默认内核：性能与功耗管理均衡               | 通用桌面/服务器            |
| `lowlatency`        | 降低中断延迟；更好的调度响应性             | 机器人、音频处理、软实时   |
| `rt`（`PREEMPT_RT`）| 完全可抢占；硬实时确定性                   | 工业控制、关键任务系统     |

---

## ⚠️ 重要免责声明

升级到**低延迟**或**实时（RT）内核**可能会：
- 破坏专有驱动（如 NVIDIA、Wi-Fi 模块）
- 导致系统不稳定或无法启动

**你需自行承担**因内核变更引起的任何问题的全部责任。
✅ 在继续之前**务必备份系统**。

---


## 为 Ubuntu/x86-64 安装低延迟或 PREEMPT_RT 内核

### 选项 1：低延迟内核

1. **安装内核**：
   对于 Hardware Enablement (HWE) 堆栈（用 `uname -r` 检查；例如内核为 6.x 的 Ubuntu 22.04），使用：

   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency-hwe-22.04  # 将 "22.04" 替换为你的版本
   ```
   对于原版内核（5.15），使用：
   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency
   ```

2. **设置 GRUB 优先使用低延迟内核**：
   ```bash
    echo 'GRUB_FLAVOUR_ORDER="lowlatency"' | sudo tee -a /etc/default/grub
    sudo update-grub
   ```

3. **重启并验证**：
    ```bash
    sudo reboot
    uname -r  # 应显示 "...-lowlatency"
    ```

  🔄 要恢复为 ``generic``，将 ``GRUB_FLAVOUR_ORDER`` 改为 ``"generic"`` 并运行 ``sudo update-grub``。

### 选项 2：PREEMPT_RT 内核

本页列出的 Ubuntu 发行版原生支持实时内核，可通过几条命令轻松启用，完整教程见 https://ubuntu.com/real-time 。


> ℹ️ Ubuntu 22.04/24.04 用户可通过**免费的 Ubuntu Pro 订阅**启用 RT 内核。该订阅对个人使用免费。

> ℹ️ 对于 Nvidia Jetson（**aarch64**），请参阅 Nvidia 官方文档。

---
