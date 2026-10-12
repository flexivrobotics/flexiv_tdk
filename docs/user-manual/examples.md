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

## C++ Examples
Location: `example/`

Examples include:
- `transparent_cartesian_teleop_lan.cpp`
- `transparent_cartesian_teleop_wan.cpp`
- `transparent_cartesian_teleop_wan_pro.cpp`
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

Install **TDK v1.6.4** first. It requires **`flexivrdk==1.9.4`** (pulled in automatically):

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
