# 時刻同期（WAN）

正確な時刻同期は WAN 遠隔操作に不可欠です。インターネット経由で操作する場合、各エッジコンピュータのシステム時刻をキャリブレーションする必要があります。

## 1) Chrony のインストールと起動

```bash
sudo apt install chrony -y
systemctl status chrony  # "active (running)" と表示されるはず
```

## 2) 同期精度の確認

```bash
chronyc tracking | grep 'System time\|RMS offset'
```

| 指標        | 良好 (ms) | 許容 (ms) | 不良 (ms) |
| ----------- | --------- | --------- | --------- |
| System time | < 1       | 1 - 10    | > 10      |
| RMS offset  | < 5       | 5 - 20    | > 20      |

## 3) 即時同期の強制（必要な場合）

```bash
sudo chronyc burst 4/4
sudo chronyc makestep
```

ネットワーク変更後（例：Wi-Fi → 有線 LAN）は、再起動してください：

```bash
sudo systemctl restart chronyd
sleep 5
sudo chronyc makestep
```

詳細：[chrony](https://chrony-project.org/)
