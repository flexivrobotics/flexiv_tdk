# Robots setup and network configuration

!!! warning "Caution"

    Before proceeding, make sure the robot is securely mounted onto a steady base and won’t topple over when moving at a high speed with sudden stops.

TDK supports both local area network (LAN) and wide area network (WAN) deployments. The following section uses a LAN setup as an example to describe the network configuration for the robots and the user computer.

## Step 1: Power on and servo on the robots
Follow instructions in the provided Quick Start Guide to complete hardware setup and boot up the two robots. After booting up, connect the robots to the UI tablet (Flexiv Elements). All the light rings on the robots should be deep blue if they are powered on and servo on.

## Step 2: Enable Remote Mode - Ethernet

Go to Settings > Remote Mode on the Flexiv Elements.
![settings_remote_mode](../assets/settings_remote_mode.png)
To enable Remote Mode, turn on the toggle, then select Ethernet from the Select a Mode drop-down list
![remote_mode_ethernet](../assets/remote_mode_ethernet.png)

View the currently applied Ethernet communication protocol(s). If no license has been installed, you can install the TDK license in Settings >
License.
![ethernet_license](../assets/ethernet_license.png)

## Step 3: Configure the network for leader robot and follower robot

The network configuration for the leader robot and follower robot can be done via the ``Flexiv Elements->Settings->Robot Control Box``.
Please refer to Flexiv Elements User Manual for more details.
![control_box_net_cfg](../assets/control_box_network.png)

For LAN teleoperation, please set leader and follower robots and the user's computer to the same network segment with different IP Addresses (e.g., leader robot: 192.168.2.110, follower robot: 192.168.2.111 and user PC:192.168.2.112. All configured with a subnet mask of 255.255.255.0).

## Step 4: Reboot the robots and ping test
After configuring the network and remote mode, reboot the robots. Connect the user's computer and two robots to the same ethernet switch. Ping the two control boxes to make sure all the devices are connected properly.

## Step 5: Tool calibration
Following `Calibrate Tool` chapter in Flexiv Elements User Manual, make sure inertia parameters of the tools on both robots are calibrated and the configured tools are indeed the current ones. Set the TCP Position of the leader robot at the position where the human hand actually grips.

