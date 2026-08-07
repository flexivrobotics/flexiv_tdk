# 時間同步（WAN）

精確的時間同步對 WAN 遙操作至關重要。透過網際網路操作時，每台邊緣電腦的系統時間都必須校準。

## 1) 安裝並啟動 Chrony

```bash
sudo apt install chrony -y
systemctl status chrony  # 應顯示 "active (running)"
```

## 2) 檢查同步精度

```bash
chronyc tracking | grep 'System time\|RMS offset'
```

| 指標        | 良好 (ms) | 可接受 (ms) | 較差 (ms) |
| ----------- | --------- | ----------- | --------- |
| System time | < 1       | 1 - 10      | > 10      |
| RMS offset  | < 5       | 5 - 20      | > 20      |

## 3) 強制立即同步（如需要）

```bash
sudo chronyc burst 4/4
sudo chronyc makestep
```

網路變更後（例如 Wi-Fi → 乙太網路），請重新啟動：

```bash
sudo systemctl restart chronyd
sleep 5
sudo chronyc makestep
```

了解更多：[chrony](https://chrony-project.org/)
