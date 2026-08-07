# Schnellstart (Python)

Diese Anleitung führt Sie durch die Installation und Ausführung des Flexiv-TDK-Python-Pakets. Die Pakete wurden auf [PyPI](https://pypi.org/project/flexivtdk/) veröffentlicht.

## 1) Paket installieren

```bash
python3.x -m pip install spdlog flexivtdk
```

> Ersetzen Sie `3.x` durch Ihre Python-Version (z. B. 3.10).

## 2) Installation überprüfen

```bash
python3.x
>>> import flexivtdk
>>> flexivtdk.__version__
```

## 3) Python-Beispiele aus diesem Repository ausführen

Damit ein regulärer Benutzer hochpriore (Echtzeit-)Threads ohne `sudo` erstellen kann, konfigurieren Sie das System so, dass Echtzeit- und Nice-Prioritätsgrenzen angewendet werden (nur einmal erforderlich):

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

Melden Sie sich ab und wieder an (oder starten Sie neu), damit die Einstellungen wirksam werden. Danach können alle Beispiele ohne `sudo` ausgeführt werden.

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

Details zur Verwendung finden Sie im Quellcode der jeweiligen Beispiele.

!!! warning "Wichtig"

    Beachten Sie, dass die Mensch-Roboter-Interaktion aus dem TCP-Kraftmoment gelesen wird. Greifen Sie daher nicht mit einer Hand den Endeffektor des Roboters und mit der anderen Hand ein anderes Glied des Roboters – dies ist ein häufiger Bedienfehler.


## Nützliche Tipps

- Stellen Sie eine stabile Netzwerkverbindung zum Roboter sicher.
- Für WAN-Teleoperation siehe [Zeitsynchronisation (WAN)](time-sync.md).
- Für Echtzeitleistung siehe [Echtzeit-Kernel](real-time-kernel.md).
