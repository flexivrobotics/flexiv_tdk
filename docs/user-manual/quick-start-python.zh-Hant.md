# 快速上手（Python）

本指南將引導你安裝並執行 Flexiv TDK Python 套件。套件已發佈在 [PyPI](https://pypi.org/project/flexivtdk/) 上。

## 1) 安裝套件

```bash
python3.x -m pip install flexivtdk
```

> 將 `3.x` 替換為你的 Python 版本。Linux 支援 3.10、3.12 和 3.14；macOS 支援 3.10 和 3.12。

## 2) 驗證安裝

```bash
python3.x
>>> import flexivtdk
>>> flexivtdk.__version__
```

## 3) 執行本儲存庫中的 Python 範例

為允許一般使用者在不使用 `sudo` 的情況下建立高優先級（即時）執行緒，請配置系統套用即時和 nice 優先級限制（只需設定一次）：

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

登出並重新登入（或重新開機）使設定生效。之後所有範例都可以在不使用 `sudo` 的情況下執行。

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

用法詳情請查看各範例的原始碼。

!!! warning "重要"

    請注意，人機互動力是從 TCP 力/力矩讀取的。因此，不要一隻手握住機器人末端執行器、另一隻手握住機器人的其他連桿——這是一個常見的操作錯誤。


## 常用提示

- 確保機器人網路連接穩定。
- WAN 遙操作請參見[時間同步 (WAN)](time-sync.md)。
- 即時效能請參見[即時核心](real-time-kernel.md)。
