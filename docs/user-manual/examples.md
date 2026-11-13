# Examples

The repository includes example applications for both C++ and Python.

This release uses `NetworkCfgStd` for WAN Standard Edition TCP peer-to-peer. The WAN `-W` / `--wan-iface` argument is an OS interface name (for example `wlo1` or `enp3s0`), not an IPv4 address. `-A` / `--lan-ip` remains the LAN IPv4 address of the NIC connected to the robot.

Console commands added in v1.6.4:

- `h`: print `GetTeleopStatus()` (why teleop is restricted or paused, and what to do next)
- WAN `n`: print `role()` and `robot_pair_sn()`
- LAN `H`: `HomeAll()`
- `quit` (LAN and WAN transparent examples): stop teleop and exit the program

## WAN Professional Edition example

`transparent_cartesian_teleop_wan_pro` (C++ and Python) uses `NetworkCfgPro` instead of TCP peer-to-peer. Both sides connect through a TDK Server deployment.

- `-c <client.conf>`: path to `client.conf` from the TDK Server credential package (required).
- `-n <gripper_name>`: optional. Name of the follower gripper configured in Elements -> Device. When provided, the example also creates a `GripperRemoteControl` instance. Both leader and follower must pass the same name. The follower enables the gripper locally at startup; the leader controls it with the console commands `G`/`D`/`N` (enable, disable, init), `o`/`c`/`s` (open, close, stop), and `t`/`P` (print states and params).
- `-D`: optional, leader only. DI0 engages arm teleop. With `-n`, a rising edge on DI1 toggles the follower gripper between open and closed, and the console gripper motion commands `o`/`c`/`s` are disabled.

## Teleop over WiFi example

`transparent_cartesian_teleop_wifi` (C++) runs single-arm or dual-arm transparent Cartesian teleop between two computers on the same WiFi network. Each computer is wired to its own robots: the leader computer to the leader robots, the follower computer to the follower robots. Only the teleop messages between the two computers travel over WiFi, through one TCP connection (`NetworkCfgStd`). No public IP address or port forwarding is needed.

- `-l`/`-f`: leader/follower serial numbers of pair 0.
- `-L`/`-F`: optional. Leader/follower serial numbers of pair 1, for dual-arm teleop. Pass both or neither.
- Both computers pass the same serial numbers in the same order.
- `-i <ip>`: WiFi IPv4 address of the TCP server computer, on both sides.
- `-A <lan_ipv4>`: IPv4 address of the NIC wired to the robots, so robot traffic stays off WiFi.
- `-W <iface>`: WiFi interface name (for example `wlo1`), so the teleop link stays on WiFi.
- `-T <ms>`: optional. Maximum acceptable message latency, 0-350 ms (default 200).
- `-D`: optional, leader only. DI0 on the pair 0 leader robot engages and disengages all pairs.
- Console commands act on all pairs. With two pairs, append `0` or `1` to act on one pair only, for example `u1` restarts pair 1. `m` toggles printing the message latency every second.

Leader computer (TCP server), dual-arm:

```bash
sudo ./transparent_cartesian_teleop_wifi -l <L0> -f <F0> -L <L1> -F <F1> -r leader -t server -i <server_wifi_ip> -p 50000 -A <wired_ipv4> -W <wifi_iface> -D
```

Follower computer (TCP client), dual-arm:

```bash
sudo ./transparent_cartesian_teleop_wifi -l <L0> -f <F0> -L <L1> -F <F1> -r follower -t client -i <server_wifi_ip> -p 50000 -A <wired_ipv4> -W <wifi_iface>
```

For single-arm teleop, drop `-L <L1> -F <F1>` on both computers.

Before running:

1. Use a WiFi network without AP/client isolation. Guest and corporate networks often block traffic between devices; a dedicated 5 GHz router works best. Its subnet must not overlap the robot subnet.
2. Turn off WiFi power saving on both computers: `sudo iw dev <wifi_iface> set power_save off`.
3. Open the port on the server computer, for example `sudo ufw allow 50000/tcp`.
4. Synchronize the two clocks, see [Time Sync (WAN)](time-sync.md). Without Internet access, make the server computer the time source: add `local stratum 8` and `allow <wifi_subnet>` to its `/etc/chrony/chrony.conf`, add `server <server_wifi_ip> iburst prefer` on the client, then restart chrony on both. Local WiFi latency is only a few milliseconds, so even a small clock offset reads as negative latency and forces teleop to disengage.

## C++ Examples
Location: `example/`

Examples include:
- `transparent_cartesian_teleop_lan.cpp`
- `transparent_cartesian_teleop_wan.cpp`
- `transparent_cartesian_teleop_wan_pro.cpp`
- `transparent_cartesian_teleop_wifi.cpp`
- `joint_teleop_under_lan.cpp`

Build and run:

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
./<program_name> [arguments]
```

If the loader cannot find `libflexiv_tdk` / `libflexiv_rdk`:

```bash
# Linux
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]

# macOS
DYLD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```

## Python Examples
Location: `example_py/`

Examples include:
- `transparent_cartesian_teleop_lan.py`
- `transparent_cartesian_teleop_wan.py`
- `transparent_cartesian_teleop_wan_pro.py`
- `joint_teleop_under_lan.py`

Install **TDK v1.6.4** first. It requires **`flexivrdk==1.9.4.1`** (pulled in automatically):

```bash
python3.x -m pip install flexivtdk==1.6.4
```

Some console commands in examples (for example LAN `P`, which calls `instances()`) use the underlying RDK `Robot` objects and need that matching `flexivrdk` package.

Run:

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

> Replace `3.x` with your Python version.
