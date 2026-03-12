# Examples

The repository includes example applications for both C++ and Python.

## C++ Examples
Location: `example/`

Examples include:
- `cartesian_teleop_under_lan.cpp`
- `joint_teleop_under_lan.cpp`
- `joint_teleop_over_wan.cpp`
- `transparent_cartesian_teleop_lan.cpp`
- `transparent_cartesian_teleop_wan.cpp`

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
- `cartesian_teleop_under_lan.py`
- `joint_teleop_under_lan.py`
- `joint_teleop_over_wan.py`
- `transparent_cartesian_teleop_lan.py`
- `transparent_cartesian_teleop_wan.py`

Run:

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

> Replace `3.x` with your Python version.
