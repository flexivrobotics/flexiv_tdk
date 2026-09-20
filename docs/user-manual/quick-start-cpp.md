# Quick Start (C++)

This guide covers building and using Flexiv TDK as a CMake package.

**Version mapping:** TDK **v1.6.4** is a self-contained shared library. The only public C++ dependency is **flexiv_rdk 1.9.4**. Third-party libraries are embedded and hidden. `thirdparty/build_and_install_dependencies.sh` installs RDK.

## 1) Install build dependencies

```bash
# Linux
sudo apt install build-essential cmake cmake-qt-gui -y

# macOS (Xcode command-line tools + Homebrew CMake)
xcode-select --install
brew install cmake
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

CMake records the install `lib` directory in the example's rpath, so Linux and macOS can run the binary directly:

```bash
cd flexiv_tdk/example/build
./<program_name> [arguments]
```

TDK looks up `libflexiv_rdk` next to itself (`$ORIGIN` / `@loader_path`). The install `lib` directory must contain both libraries (`.so` on Linux, `.dylib` on macOS). Fallback:

```bash
# Linux
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]

# macOS
DYLD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```

See [Examples](examples.md) for details on the example programs.
