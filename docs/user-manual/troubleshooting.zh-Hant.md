# 疑難排解

## 建構問題

- **CMake 找不到 Flexiv TDK**：確保 `-DCMAKE_PREFIX_PATH=~/tdk_install` 指向安裝前綴。
- **缺少依賴**：重新執行 `thirdparty/build_and_install_dependencies.sh`，並確保安裝目錄存在。

## 執行時問題

- **找不到共享函式庫**：執行二進位檔案前設定 `LD_LIBRARY_PATH=~/tdk_install/lib`。
- **即時優先級權限問題**：在 `/etc/security/limits.conf` 中為你的使用者新增 `rtprio` 和 `nice` 配置。

## WAN 遙操作

- **高延遲 / 運動不穩定**：檢查網路穩定性，並重新檢查時間同步。
- **時間同步不準確**：使用 `chronyc tracking` 檢查，並考慮重新執行 `chronyc burst` 和 `chronyc makestep`。

## 主端機器人漂移

- 檢查主端機器人工具的慣量參數標定，確認所配置的工具確實是目前使用的工具。
- 檢查主端機器人的 TCP 位置是否設定在人手實際握持的位置。
- 啟動 TDK 範例時，是否標定感測器由實際輸入參數決定。如果選擇在初始化時標定感測器，標定會持續一段時間，在此過程中請勿觸碰設備。

## 獲取幫助

- [GitHub Issues](https://github.com/flexivrobotics/flexiv_tdk/issues)
