# 时间同步（WAN）

精确的时间同步对 WAN 遥操作至关重要。通过互联网操作时，每台边缘计算机的系统时间都必须校准。

## 1) 安装并启动 Chrony

```bash
sudo apt install chrony -y
systemctl status chrony  # 应显示 "active (running)"
```

## 2) 检查同步精度

```bash
chronyc tracking | grep 'System time\|RMS offset'
```

| 指标        | 良好 (ms) | 可接受 (ms) | 较差 (ms) |
| ----------- | --------- | ----------- | --------- |
| System time | < 1       | 1 - 10      | > 10      |
| RMS offset  | < 5       | 5 - 20      | > 20      |

## 3) 强制立即同步（如需要）

```bash
sudo chronyc burst 4/4
sudo chronyc makestep
```

网络变更后（例如 Wi-Fi → 以太网），请重启：

```bash
sudo systemctl restart chronyd
sleep 5
sudo chronyc makestep
```

了解更多：[chrony](https://chrony-project.org/)
