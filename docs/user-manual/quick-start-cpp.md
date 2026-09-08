# Quick Start (C++)

This guide covers building and using Flexiv TDK as a CMake package.

## 1) Install build dependencies

```bash
sudo apt install build-essential cmake cmake-qt-gui -y
```

## 2) Choose an install directory

Example:

```bash
mkdir -p ~/tdk_install
```

## 3) Build and install third-party dependencies

```bash
cd flexiv_tdk/thirdparty
bash build_and_install_dependencies.sh ~/tdk_install
```

> Internet access to GitHub is required.

## 4) Configure and install TDK

```bash
cd flexiv_tdk
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install
cmake --build . --target install --config Release
```

## 5) Link TDK in your project

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
```

## 6) Run examples

```bash
cd flexiv_tdk/example/build
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```

`LD_LIBRARY_PATH` must include the directory that contains both `libflexiv_tdk.so` and `libflexiv_rdk.so`.

To allow a regular user to create high-priority (real-time) threads without `sudo`, configure system to apply real-time and nice priority limits (only need to set it once):

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```
Log out and log back in (or reboot) for the settings to take effect. Then all the examples can be executed without `sudo`.

See [API Reference](../api/doxygen/index.html) for details.

