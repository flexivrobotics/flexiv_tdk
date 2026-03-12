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

> `LD_LIBRARY_PATH` specifies where shared libraries for dependencies are installed.

See [Examples](examples.md) for details on the example programs.
