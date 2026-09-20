# Q&A

## Q: Which platforms are supported?
**A:** Ubuntu 22.04+ on x86_64 and aarch64 (C++ and Python 3.10/3.12/3.14). macOS 14+ on arm64 (C++ and Python 3.10/3.12). Toolchain: GCC ≥ 9.4 (Linux) or Apple Clang ≥ 15 (macOS), CMake ≥ 3.16.3.

| TDK | C++ RDK | Python RDK |
| --- | ------- | ---------- |
| **v1.6.4** (`flexivtdk==1.6.4`) | **flexiv_rdk 1.9.4** | **flexivrdk==1.9.4** |

`pip install flexivtdk==1.6.4` installs `flexivrdk==1.9.3` automatically. Do not mix TDK 1.6.4 with another `flexivrdk` version.

## Q: Do I need a real-time kernel?
**A:** Not strictly, but a low-latency or RT kernel improves responsiveness and stability for teleoperation. See [Real-Time Kernel](../user-manual/real-time-kernel.md).

## Q: How do I sync time for WAN teleoperation?
**A:** Use Chrony to sync system clocks on both ends. See [Time Sync (WAN)](../user-manual/time-sync.md).

## Q: Where is the API reference?
**A:** The online GitHub Pages site is TDK v2.x. For this v1.6.x tree, generate Doxygen locally (`doxygen docs/doxygen/Doxyfile.in`) and serve the site with `mkdocs serve`. See the repository README.

## Q: What if my app cannot find shared libraries?
**A:** Set `LD_LIBRARY_PATH=~/tdk_install/lib` before running binaries.

## Q: Where can I get help?
**A:** Open an issue on GitHub or contact Flexiv via https://www.flexiv.com/contact.
