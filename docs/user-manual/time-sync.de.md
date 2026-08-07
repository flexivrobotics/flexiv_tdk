# Zeitsynchronisation (WAN)

Eine genaue Zeitsynchronisation ist für WAN-Teleoperation entscheidend. Beim Betrieb über das Internet muss die Systemzeit jedes Edge-Computers kalibriert werden.

## 1) Chrony installieren und starten

```bash
sudo apt install chrony -y
systemctl status chrony  # sollte "active (running)" anzeigen
```

## 2) Synchronisationsgenauigkeit prüfen

```bash
chronyc tracking | grep 'System time\|RMS offset'
```

| Metrik      | Gut (ms) | Akzeptabel (ms) | Schlecht (ms) |
| ----------- | -------- | --------------- | ------------- |
| System time | < 1      | 1 - 10          | > 10          |
| RMS offset  | < 5      | 5 - 20          | > 20          |

## 3) Sofortige Synchronisation erzwingen (falls erforderlich)

```bash
sudo chronyc burst 4/4
sudo chronyc makestep
```

Nach Netzwerkänderungen (z. B. Wi-Fi → Ethernet) neu starten:

```bash
sudo systemctl restart chronyd
sleep 5
sudo chronyc makestep
```

Mehr erfahren: [chrony](https://chrony-project.org/)
