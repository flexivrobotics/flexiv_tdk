# 机器人设置与网络配置

!!! warning "注意"

    在继续之前，请确保机器人牢固安装在稳固的基座上，在高速运动并急停时不会倾覆。

TDK 同时支持局域网（LAN）和广域网（WAN）部署。本节以 LAN 部署为例，介绍机器人和用户计算机的网络配置。

## 第 1 步：机器人上电并使能

按照随附的《快速入门指南》完成硬件安装并启动两台机器人。启动后，将机器人连接到 UI 平板（Flexiv Elements）。机器人上电并使能后，所有灯环应呈深蓝色。

## 第 2 步：启用远程模式 - Ethernet

在 Flexiv Elements 上进入"设置 > 远程模式"。
![settings_remote_mode](../assets/settings_remote_mode.png)
要启用远程模式，打开开关，然后从"选择模式"下拉列表中选择 Ethernet。
![remote_mode_ethernet](../assets/remote_mode_ethernet.png)

查看当前已应用的以太网通信协议。如果尚未安装许可证，可以在"设置 > 许可证"中安装 TDK 许可证。
![ethernet_license](../assets/ethernet_license.png)

## 第 3 步：配置主端和从端机器人的网络

主端机器人和从端机器人的网络配置可通过 ``Flexiv Elements->设置->机器人控制箱`` 完成。
更多详情请参阅《Flexiv Elements 用户手册》。
![control_box_net_cfg](../assets/control_box_network.png)

对于 LAN 遥操作，请将主端机器人、从端机器人和用户计算机设置在同一网段并使用不同的 IP 地址（例如：主端机器人 192.168.2.110，从端机器人 192.168.2.111，用户计算机 192.168.2.112，子网掩码均为 255.255.255.0）。

## 第 4 步：重启机器人并 ping 测试

完成网络和远程模式配置后，重启机器人。将用户计算机和两台机器人连接到同一个以太网交换机。Ping 两个控制箱，确认所有设备连接正常。

## 第 5 步：工具标定

按照《Flexiv Elements 用户手册》中"工具标定"章节，确保两台机器人上工具的惯量参数都已标定，且所配置的工具确实是当前使用的工具。将主端机器人的 TCP 位置设置在人手实际握持的位置。
