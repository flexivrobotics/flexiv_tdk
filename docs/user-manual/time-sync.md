# Time Sync (WAN)

Accurate time synchronization is critical for WAN teleoperation. When operating over the internet, each edge computer must have its system time calibrated.

## 1) Install and start Chrony

```bash
sudo apt install chrony -y
systemctl status chrony  # should show "active (running)"
```

## 2) Check sync accuracy

```bash
chronyc tracking | grep 'System time\|RMS offset'
```

| Metric      | Good (ms) | Acceptable (ms) | Poor (ms) |
| ----------- | --------- | --------------- | --------- |
| System time | < 1       | 1 - 10          | > 10      |
| RMS offset  | < 5       | 5 - 20          | > 20      |

## 3) Force immediate sync (if needed)

```bash
sudo chronyc burst 4/4
sudo chronyc makestep
```

After network changes (e.g., Wi-Fi → Ethernet), restart:

```bash
sudo systemctl restart chronyd
sleep 5
sudo chronyc makestep
```

Learn more: [chrony](https://chrony-project.org/)
