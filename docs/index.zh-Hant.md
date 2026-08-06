# Flexiv's TDK | Teleoperation Made Simple

歡迎來到 **Flexiv TDK（遙操作開發套件）** 文件站點。本站提供使用者手冊、API 參考和常見問題解答，幫助你建構並整合 TDK。

## 什麼是 Flexiv TDK？

Flexiv TDK 是一套 SDK，用於基於 Flexiv 自適應機器人建構自訂的「機器人對機器人」或「設備對機器人」遙操作應用。它透過**高保真感知回饋**實現同步的力引導運動，並同時支援 **LAN**（區域網路）和 **WAN**（廣域網路）連接。

TDK 在具身智慧資料採集、放射醫療、危化品實驗等領域擁有廣泛的應用場景。

🎬 **[Flexiv's TDK | Teleoperation Made Simple](https://www.youtube.com/watch?v=H0e9FSZIa14)**
*（點擊下方圖片播放）*
<p align="center">
  <a href="https://www.youtube.com/watch?v=H0e9FSZIa14" target="_blank">
    <img src="https://img.youtube.com/vi/H0e9FSZIa14/hqdefault.jpg" alt="TDK Demo 1" width="350" style="margin-right:10px;" />
  </a>
  <a href="https://www.youtube.com/watch?v=udkddqxth5Q" target="_blank">
    <img src="https://img.youtube.com/vi/udkddqxth5Q/hqdefault.jpg" alt="TDK Demo 2" width="350" />
  </a>
</p>

## 接觸作業操作基準測試

Flexiv TDK 已入選 [Manipulation Net 插孔裝配排行榜](https://manipulation-net.org/leaderboards/peg_in_hole.html)。[Manipulation Net](https://manipulation-net.org) 是一個面向真實世界、規模化、任意機器人和任意時間地點的公開操作基準測試。這為 TDK 相關應用場景（包括柔順插裝、對準和力覺遙操作工作流程）所需的接觸作業操作能力提供了外部佐證。

## 核心特性

- **高保真感知回饋**：100% 觸覺回饋透明度，確保人手操作的保真度。
- ****：靈活的網路配置，同時支援區域網路和廣域網路。
- **更優的物理人機互動**：主端機器人可隨時改變位置或朝向；重新接合時僅映射相對運動，無絕對位姿約束。
- **可選笛卡爾約束**：可沿指定方向約束運動，任務執行更快、更精確。
- **穩健的力/力矩保護**：防止機器人和工件受損，確保接觸過程中的本質安全。

## 快速連結

- [使用者手冊](user-manual/overview.md)
- [API 參考](../api/doxygen/index.html)
- [常見問題](qa/index.md)
- [GitHub 儲存庫](https://github.com/flexivrobotics/flexiv_tdk)

## 文件結構

- **使用者手冊**：安裝設定、範例和操作指導。
- **API 參考**：由 Doxygen 產生的 C++ API 文件。
- **常見問題**：常見問題和疑難排解技巧。

> 注：API 參考由 Doxygen 產生，並與本站一同發佈在 GitHub Pages 上。
