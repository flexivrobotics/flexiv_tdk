# Q&A

## Q: Which platforms are supported?
**A:** Ubuntu 22.04+ on x86_64 and aarch64 (C++ and Python 3.10/3.12/3.14). macOS 14+ on arm64 (C++ and Python 3.10/3.12). Toolchain: GCC ≥ 11 (Linux) or Apple Clang ≥ 15 (macOS), CMake ≥ 3.16.3.

| TDK | C++ RDK | Python RDK |
| --- | ------- | ---------- |
| **v1.7** (`flexivtdk==1.7`) | **flexiv_rdk 1.9.4.1** | **flexivrdk==1.9.4.1** |

`pip install flexivtdk==1.7` installs `flexivrdk==1.9.4.1` automatically. Do not mix TDK 1.7 with another `flexivrdk` version.

## Q: Do I need a real-time kernel?
**A:** Not strictly, but a low-latency or RT kernel improves responsiveness and stability for teleoperation. See [Real-Time Kernel](../user-manual/real-time-kernel.md).

## Q: How do I sync time for WAN teleoperation?
**A:** Use Chrony to sync system clocks on both ends. See [Time Sync (WAN)](../user-manual/time-sync.md).

## Q: Where is the API reference?
**A:** Open the [C++ API Reference](../api/index.md) and choose your release from the version dropdown. Do not use the TDK v2 API pages with this release. To preview this tree locally, see the repository README.

## Q: What if my app cannot find shared libraries?
**A:** Set `LD_LIBRARY_PATH=~/tdk_install/lib` (Linux) or `DYLD_LIBRARY_PATH=~/tdk_install/lib` (macOS) before running binaries.

## Q: Where can I get help?
**A:** Open an issue on GitHub or contact Flexiv via https://www.flexiv.com/contact.
