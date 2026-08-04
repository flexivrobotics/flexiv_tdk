# 常見問題

## 問：支援哪些平台？
**答：** x86_64 和 aarch64 架構的 Ubuntu 22.04+。支援 C++ 和 Python，要求 GCC ≥ 9.4、CMake ≥ 3.16.3。

## 問：我需要即時核心嗎？
**答：** 不是必須的，但低延遲或 RT 核心可以提升遙操作的回應性和穩定性。參見[即時核心](../user-manual/real-time-kernel.md)。

## 問：WAN 遙操作如何進行時間同步？
**答：** 使用 Chrony 同步兩端的系統時鐘。參見[時間同步 (WAN)](../user-manual/time-sync.md)。

## 問：API 參考在哪裡？
**答：** Doxygen API 參考發佈在 GitHub Pages 的 `api/doxygen/index.html` 路徑下。參見 [API 參考](../../api/doxygen/index.html)。

## 問：在哪裡可以獲得幫助？
**答：** 聯繫你的銷售經理，在 GitHub 上提交 issue，或透過 https://www.flexiv.com/contact 聯繫 Flexiv。
