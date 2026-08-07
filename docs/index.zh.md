# Flexiv's TDK | Teleoperation Made Simple

欢迎来到 **Flexiv TDK（遥操作开发套件）** 文档站点。本站提供用户手册、API 参考和常见问题解答，帮助你搭建并集成 TDK。

## 什么是 Flexiv TDK？

Flexiv TDK 是一套 SDK，用于基于 Flexiv 自适应机器人构建自定义的"机器人对机器人"或"设备对机器人"遥操作应用。它通过**高保真感知反馈**实现同步的力引导运动，并同时支持 **LAN**（局域网）和 **WAN**（广域网）连接。

TDK 在具身智能数据采集、放射医疗、危化品实验等领域拥有广泛的应用场景。

## TDK 实际应用

<div class="tdk-carousel" markdown="1">
  <div class="tdk-carousel-viewport" markdown="1">

![Physical AI](assets/carousel/Physical AI.png)

![医疗](assets/carousel/Medical.png)

![教育培训](assets/carousel/Education.png)

![危险操作](assets/carousel/Hazardous.png)

  </div>
  <button class="tdk-carousel-btn tdk-carousel-prev" aria-label="Previous image">&#8249;</button>
  <button class="tdk-carousel-btn tdk-carousel-next" aria-label="Next image">&#8250;</button>
  <div class="tdk-carousel-dots"></div>
</div>

🎬 **[Flexiv's TDK | Teleoperation Made Simple](https://www.bilibili.com/video/BV1u5ySBAE88/)**
<div class="tdk-videos">
  <a class="tdk-video-card bilibili" href="https://www.bilibili.com/video/BV1u5ySBAE88/" target="_blank" rel="noopener">
    <img src="../assets/videos/bv1u5ysbae88.jpg" alt="硬核登场！机器人遥操作开发工具包 Flexiv TDK" loading="lazy" />
  </a>
  <a class="tdk-video-card bilibili" href="https://www.bilibili.com/video/BV18yg7zkEuG/" target="_blank" rel="noopener">
    <img src="../assets/videos/bv18yg7zkeug.jpg" alt="感知同步 · 触手可达：解锁非夕自适应机器人遥操作" loading="lazy" />
  </a>
</div>

## 接触作业操作基准测试

Flexiv TDK 已入选 [Manipulation Net 插孔装配排行榜](https://manipulation-net.org/leaderboards/peg_in_hole.html)。[Manipulation Net](https://manipulation-net.org) 是一个面向真实世界、规模化、任意机器人和任意时间地点的公开操作基准测试。这为 TDK 相关应用场景（包括柔顺插装、对准和力觉遥操作工作流）所需的接触作业操作能力提供了外部佐证。

## 核心特性

- **高保真感知反馈**：100% 触觉反馈透明度，确保人手操作的保真度。
- **跨國家遠距離遙操作**：灵活的网络配置，同时支持局域网和广域网。
- **更优的物理人机交互**：主端机器人可随时改变位置或朝向；重新接合时仅映射相对运动，无绝对位姿约束。
- **可选笛卡尔约束**：可沿指定方向约束运动，任务执行更快、更精确。
- **鲁棒的力/力矩保护**：防止机器人和工件受损，确保接触过程中的本质安全。

## 快速链接

- [用户手册](user-manual/overview.md)
- [API 参考](../api/doxygen/index.html)
- [常见问题](qa/index.md)
- [GitHub 仓库](https://github.com/flexivrobotics/flexiv_tdk)

## 文档结构

- **用户手册**：安装设置、示例和操作指导。
- **API 参考**：由 Doxygen 生成的 C++ API 文档。
- **常见问题**：常见问题和故障排查技巧。

> 注：API 参考由 Doxygen 生成，并与本站一同发布在 GitHub Pages 上。
