# 快速上手（C++）

本指南介绍如何将 Flexiv TDK 作为 CMake 软件包构建和使用。

## 1) 安装构建依赖

```bash
sudo apt install build-essential cmake cmake-qt-gui -y
```

## 2) 选择安装目录

示例：

```bash
mkdir -p ~/tdk_install
```

## 3) 构建并安装第三方依赖

```bash
cd flexiv_tdk/thirdparty
bash build_and_install_dependencies.sh ~/tdk_install
```

> 需要能够访问 GitHub 的网络连接。

## 4) 配置并安装 TDK

```bash
cd flexiv_tdk
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install
cmake --build . --target install --config Release
```

## 5) 在你的项目中链接 TDK

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
```

## 6) 运行示例

CMake 会在链接时把安装目录的 `lib` 写入示例的 rpath，因此 Linux 和 macOS 都可以直接运行：

```bash
cd flexiv_tdk/example/build
./<program_name> [arguments]
```

此配置下不需要设置 `LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH`。TDK 会在自身所在目录查找 `libflexiv_rdk`（`$ORIGIN` / `@loader_path`）。

如果动态加载器仍然找不到 `libflexiv_tdk` 或 `libflexiv_rdk`，请用安装 TDK 和 RDK 时**同一个**前缀重新编译示例（`-DCMAKE_PREFIX_PATH`），不要安装到源码的 `lib/` 目录。备用方式：

```bash
# Linux
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]

# macOS
DYLD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```

安装目录的 `lib` 中必须同时包含 `libflexiv_tdk` 和 `libflexiv_rdk`（Linux 为 `.so`，macOS 为 `.dylib`）。

为允许普通用户在不使用 `sudo` 的情况下创建高优先级（实时）线程，请配置系统应用实时和 nice 优先级限制（只需设置一次）：

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

注销并重新登录（或重启）使设置生效。之后所有示例都可以在不使用 `sudo` 的情况下运行。

详情请参阅 [API 参考](../../api/doxygen/index.html)。
