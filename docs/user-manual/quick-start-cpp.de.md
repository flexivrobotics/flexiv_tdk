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

CMake schreibt das Installationsverzeichnis `lib` beim Linken in den rpath des Beispiels. Unter Linux und macOS kann die Binärdatei daher direkt ausgeführt werden:

```bash
cd flexiv_tdk/example/build
./<program_name> [arguments]
```

`LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH` ist in diesem Setup nicht erforderlich. TDK sucht `libflexiv_rdk` im eigenen Verzeichnis (`$ORIGIN` / `@loader_path`).

Wenn der Loader `libflexiv_tdk` oder `libflexiv_rdk` trotzdem nicht findet, bauen Sie die Beispiele mit `-DCMAKE_PREFIX_PATH` auf dasselbe Präfix neu, das für TDK und RDK verwendet wurde (nicht ins Quellverzeichnis `lib/` installieren). Als Fallback:

```bash
# Linux
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]

# macOS
DYLD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```

Im Installationsverzeichnis `lib` müssen sowohl `libflexiv_tdk` als auch `libflexiv_rdk` liegen (`.so` unter Linux, `.dylib` unter macOS).

Damit ein regulärer Benutzer hochpriore (Echtzeit-)Threads ohne `sudo` erstellen kann, konfigurieren Sie das System so, dass Echtzeit- und Nice-Prioritätsgrenzen angewendet werden (nur einmal erforderlich):

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

Melden Sie sich ab und wieder an (oder starten Sie neu), damit die Einstellungen wirksam werden. Danach können alle Beispiele ohne `sudo` ausgeführt werden.

Details finden Sie in der [API-Referenz](../../api/doxygen/index.html).
