# 常见问题

## 问：支持哪些平台？
**答：** x86_64 和 aarch64 架构的 Ubuntu 22.04+。支持 C++ 和 Python，要求 GCC ≥ 9.4、CMake ≥ 3.16.3。

## 问：我需要实时内核吗？
**答：** 不是必须的，但低延迟或 RT 内核可以提升遥操作的响应性和稳定性。参见[实时内核](../user-manual/real-time-kernel.md)。

## 问：WAN 遥操作如何进行时间同步？
**答：** 使用 Chrony 同步两端的系统时钟。参见[时间同步 (WAN)](../user-manual/time-sync.md)。

## 问：API 参考在哪里？
**答：** Doxygen API 参考发布在 GitHub Pages 的 `api/doxygen/index.html` 路径下。参见 [API 参考](../../api/doxygen/index.html)。

## 问：在哪里可以获得帮助？
**答：** 联系你的销售经理，在 GitHub 上提交 issue，或通过 https://www.flexiv.com/contact 联系 Flexiv。
