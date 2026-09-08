# Quick Start (Python)

This guide walks you through installing and running the Flexiv TDK Python package. The packages were published on [PyPI](https://pypi.org/project/flexivtdk/)

## 1) Install the package

```bash
python3.x -m pip install flexivtdk
```

> Replace `3.x` with your Python version (e.g., 3.10).

## 2) Verify installation

```bash
python3.x
>>> import flexivtdk
>>> flexivtdk.__version__
```

## 3) Run Python examples from this repo

To allow a regular user to create high-priority (real-time) threads without `sudo`, configure system to apply real-time and nice priority limits (only need to set it once):

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
echo "${USER} soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "${USER} hard memlock unlimited" | sudo tee -a /etc/security/limits.conf
```
Log out and log back in (or reboot) for the settings to take effect. Then all the examples can be executed without `sudo`.

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```
Check each example’s source code for usage details.

!!! warning "Important"

    Note that the human-robot interaction is read from the TCP wrench. Therefore, do not grab the end effector of the robot with one hand and another link of the robot with the other hand, which is a common operational mistake.


## Common Tips
- Ensure the robot network connection is stable.
- For WAN teleop, see [Time Sync (WAN)](time-sync.md).
- For real-time performance, see [Real-Time Kernel](real-time-kernel.md).
