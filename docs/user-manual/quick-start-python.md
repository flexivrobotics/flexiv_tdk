# Quick Start (Python)

This guide walks you through installing and running the Flexiv TDK Python package.

**Version mapping:** TDK **v1.6.4** (`flexivtdk==1.6.4`) requires RDK **v1.9.3** (`flexivrdk==1.9.3`). Do not mix this TDK release with another `flexivrdk` version.

## 1) Install the package

```bash
python3.x -m pip install flexivtdk==1.6.4
```

`flexivtdk==1.6.4` declares a hard dependency on `flexivrdk==1.9.3`, so pip installs that RDK wheel automatically.

> Replace `3.x` with your Python version. Linux supports 3.10, 3.12, and 3.14; macOS supports 3.10 and 3.12.

## 2) Verify installation

```bash
python3.x
>>> import flexivtdk
>>> flexivtdk.__version__
>>> import flexivrdk
>>> flexivrdk.__version__
```

Confirm `flexivtdk` is `1.6.4` and `flexivrdk` is `1.9.3`.

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
