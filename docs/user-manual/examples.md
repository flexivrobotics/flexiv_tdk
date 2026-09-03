# Examples

The repository includes example applications for both C++ and Python.

This release uses `NetworkCfgStd` for WAN Standard Edition TCP peer-to-peer. The WAN `-W` / `--wan-iface` argument is an OS interface name (for example `wlo1` or `enp3s0`), not an IPv4 address. `-A` / `--lan-ip` remains the LAN IPv4 address of the NIC connected to the robot.

Console commands added in v1.6.3:

- `h`: print `GetTeleopStatus()` (why teleop is restricted or paused, and what to do next)
- WAN `n`: print `role()` and `robot_pair_sn()`
- LAN `H`: `HomeAll()`

## C++ Examples
Location: `example/`

Examples include:
- `transparent_cartesian_teleop_lan.cpp`
- `transparent_cartesian_teleop_wan.cpp`
- `joint_teleop_under_lan.cpp`

Build and run:

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```

## Python Examples
Location: `example_py/`

Examples include:
- `transparent_cartesian_teleop_lan.py`
- `transparent_cartesian_teleop_wan.py`
- `joint_teleop_under_lan.py`

Run:

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

> Replace `3.x` with your Python version.
