# 故障排查

## 构建问题

- **CMake 找不到 Flexiv TDK**：确保 `-DCMAKE_PREFIX_PATH=~/tdk_install` 指向安装前缀。
- **缺少依赖**：重新运行 `thirdparty/build_and_install_dependencies.sh`，并确保安装目录存在。

## 运行时问题

- **找不到共享库**：运行二进制文件前设置 `LD_LIBRARY_PATH=~/tdk_install/lib`。
- **实时优先级权限问题**：在 `/etc/security/limits.conf` 中为你的用户添加 `rtprio` 和 `nice` 配置。

## WAN 遥操作

- **高延迟 / 运动不稳定**：检查网络稳定性，并重新检查时间同步。
- **时间同步不准确**：使用 `chronyc tracking` 检查，并考虑重新运行 `chronyc burst` 和 `chronyc makestep`。

## 主端机器人漂移

- 检查主端机器人工具的惯量参数标定，确认所配置的工具确实是当前使用的工具。
- 检查主端机器人的 TCP 位置是否设置在人手实际握持的位置。
- 启动 TDK 示例时，是否标定传感器由实际输入参数决定。如果选择在初始化时标定传感器，标定会持续一段时间，在此过程中请勿触碰设备。

## 获取帮助

- [GitHub Issues](https://github.com/flexivrobotics/flexiv_tdk/issues)
