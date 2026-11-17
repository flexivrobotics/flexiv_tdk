# 快速上手（C++）

本指南介紹如何將 Flexiv TDK 作為 CMake 套件建構和使用。

## 1) 安裝建構依賴

```bash
sudo apt install build-essential cmake cmake-qt-gui -y
```

## 2) 選擇安裝目錄

範例：

```bash
mkdir -p ~/tdk_install
```

## 3) 建構並安裝第三方依賴

```bash
cd flexiv_tdk/thirdparty
bash build_and_install_dependencies.sh ~/tdk_install
```

> 需要能夠存取 GitHub 的網路連接。

## 4) 配置並安裝 TDK

```bash
cd flexiv_tdk
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install
cmake --build . --target install --config Release
```

## 5) 在你的專案中連結 TDK

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
```

## 6) 執行範例

CMake 會在連結時把安裝目錄的 `lib` 寫入範例的 rpath，因此 Linux 和 macOS 都可以直接執行：

```bash
cd flexiv_tdk/example/build
./<program_name> [arguments]
```

此配置下不需要設定 `LD_LIBRARY_PATH` / `DYLD_LIBRARY_PATH`。TDK 會在自身所在目錄查找 `libflexiv_rdk`（`$ORIGIN` / `@loader_path`）。

如果動態載入器仍然找不到 `libflexiv_tdk` 或 `libflexiv_rdk`，請用安裝 TDK 和 RDK 時**同一個**前綴重新編譯範例（`-DCMAKE_PREFIX_PATH`），不要安裝到原始碼的 `lib/` 目錄。備用方式：

```bash
# Linux
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]

# macOS
DYLD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```

安裝目錄的 `lib` 中必須同時包含 `libflexiv_tdk` 和 `libflexiv_rdk`（Linux 為 `.so`，macOS 為 `.dylib`）。

為允許一般使用者在不使用 `sudo` 的情況下建立高優先級（即時）執行緒，請配置系統套用即時和 nice 優先級限制（只需設定一次）：

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

登出並重新登入（或重新開機）使設定生效。之後所有範例都可以在不使用 `sudo` 的情況下執行。

詳情請參閱 [API 參考](../../api/doxygen/index.html)。
