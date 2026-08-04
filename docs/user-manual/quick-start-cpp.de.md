# Schnellstart (C++)

Diese Anleitung behandelt das Erstellen und Verwenden von Flexiv TDK als CMake-Paket.

## 1) Build-Abhängigkeiten installieren

```bash
sudo apt install build-essential cmake cmake-qt-gui -y
```

## 2) Installationsverzeichnis wählen

Beispiel:

```bash
mkdir -p ~/tdk_install
```

## 3) Drittanbieter-Abhängigkeiten erstellen und installieren

```bash
cd flexiv_tdk/thirdparty
bash build_and_install_dependencies.sh ~/tdk_install
```

> Internetzugang zu GitHub ist erforderlich.

## 4) TDK konfigurieren und installieren

```bash
cd flexiv_tdk
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install
cmake --build . --target install --config Release
```

## 5) TDK in Ihrem Projekt verlinken

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
```

## 6) Beispiele ausführen

```bash
cd flexiv_tdk/example/build
sudo ./<program_name> [arguments]
```

Damit ein regulärer Benutzer hochpriore (Echtzeit-)Threads ohne `sudo` erstellen kann, konfigurieren Sie das System so, dass Echtzeit- und Nice-Prioritätsgrenzen angewendet werden (nur einmal erforderlich):

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

Melden Sie sich ab und wieder an (oder starten Sie neu), damit die Einstellungen wirksam werden. Danach können alle Beispiele ohne `sudo` ausgeführt werden.

Details finden Sie in der [API-Referenz](../api/doxygen/index.html).
