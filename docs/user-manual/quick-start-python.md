# Quick Start (Python)

This guide walks you through installing and running the Flexiv TDK Python package.

## 1) Install the package

```bash
python3.x -m pip install spdlog flexivtdk
```

> Replace `3.x` with your Python version (e.g., 3.10).

## 2) Verify installation

```bash
python3.x
>>> import flexivtdk
>>> flexivtdk.__version__
```

## 3) Run Python examples from this repo

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

Check each example’s source code for usage details.

## Common Tips
- Ensure the robot network connection is stable.
- For WAN teleop, see [Time Sync (WAN)](time-sync.md).
- For real-time performance, see [Real-Time Kernel](real-time-kernel.md).
