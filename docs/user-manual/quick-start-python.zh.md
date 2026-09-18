# 快速上手（Python）

本指南将引导你安装并运行 Flexiv TDK Python 软件包。软件包已发布在 [PyPI](https://pypi.org/project/flexivtdk/) 上。

## 1) 安装软件包

```bash
python3.x -m pip install flexivtdk
```

> 将 `3.x` 替换为你的 Python 版本（例如 3.10）。

## 2) 验证安装

```bash
python3.x
>>> import flexivtdk
>>> flexivtdk.__version__
```

## 3) 运行本仓库中的 Python 示例

为允许普通用户在不使用 `sudo` 的情况下创建高优先级（实时）线程，请配置系统应用实时和 nice 优先级限制（只需设置一次）：

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```

注销并重新登录（或重启）使设置生效。之后所有示例都可以在不使用 `sudo` 的情况下运行。

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

用法详情请查看各示例的源代码。

!!! warning "重要"

    请注意，人机交互力是从 TCP 力/力矩读取的。因此，不要一只手握住机器人末端执行器、另一只手握住机器人的其他连杆——这是一个常见的操作错误。


## 常用提示

- 确保机器人网络连接稳定。
- WAN 遥操作请参见[时间同步 (WAN)](time-sync.md)。
- 实时性能请参见[实时内核](real-time-kernel.md)。
