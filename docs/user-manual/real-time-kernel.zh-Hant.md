# 即時核心選項

並非所有使用場景都需要即時作業系統，請閱讀下文判斷你是否需要。

## ⚙️ 即時效能的核心選項

Ubuntu 提供多種面向不同工作負載的核心變體：

| 核心類型            | 說明                                       | 典型使用場景               |
| ------------------- | ------------------------------------------ | -------------------------- |
| `generic`           | 預設核心：效能與功耗管理均衡               | 通用桌面/伺服器            |
| `lowlatency`        | 降低中斷延遲；更好的排程回應性             | 機器人、音訊處理、軟即時   |
| `rt`（`PREEMPT_RT`）| 完全可搶佔；硬即時確定性                   | 工業控制、關鍵任務系統     |

---

## ⚠️ 重要免責聲明

升級到**低延遲**或**即時（RT）核心**可能會：
- 破壞專有驅動（如 NVIDIA、Wi-Fi 模組）
- 導致系統不穩定或無法啟動

**你需自行承擔**因核心變更引起的任何問題的全部責任。
✅ 在繼續之前**務必備份系統**。

---


## 為 Ubuntu/x86-64 安裝低延遲或 PREEMPT_RT 核心

### 選項 1：低延遲核心

1. **安裝核心**：
   對於 Hardware Enablement (HWE) 堆疊（用 `uname -r` 檢查；例如核心為 6.x 的 Ubuntu 22.04），使用：

   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency-hwe-22.04  # 將 "22.04" 替換為你的版本
   ```
   對於原版核心（5.15），使用：
   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency
   ```

2. **設定 GRUB 優先使用低延遲核心**：
   ```bash
    echo 'GRUB_FLAVOUR_ORDER="lowlatency"' | sudo tee -a /etc/default/grub
    sudo update-grub
   ```

3. **重新開機並驗證**：
    ```bash
    sudo reboot
    uname -r  # 應顯示 "...-lowlatency"
    ```

  🔄 要恢復為 ``generic``，將 ``GRUB_FLAVOUR_ORDER`` 改為 ``"generic"`` 並執行 ``sudo update-grub``。

### 選項 2：PREEMPT_RT 核心

本頁列出的 Ubuntu 發行版原生支援即時核心，可透過幾條指令輕鬆啟用，完整教學見 https://ubuntu.com/real-time 。


> ℹ️ Ubuntu 22.04/24.04 使用者可透過**免費的 Ubuntu Pro 訂閱**啟用 RT 核心。該訂閱對個人使用免費。

> ℹ️ 對於 Nvidia Jetson（**aarch64**），請參閱 Nvidia 官方文件。

---
