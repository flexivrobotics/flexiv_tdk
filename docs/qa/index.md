# Q&A

## Q: Which platforms are supported?
**A:** Ubuntu 22.04+ on x86_64 and aarch64. C++ and Python are supported with GCC ≥ 9.4 and CMake ≥ 3.16.3.

## Q: Do I need a real-time kernel?
**A:** Not strictly, but a low-latency or RT kernel improves responsiveness and stability for teleoperation. See [Real-Time Kernel](../user-manual/real-time-kernel.md).

## Q: How do I sync time for WAN teleoperation?
**A:** Use Chrony to sync system clocks on both ends. See [Time Sync (WAN)](../user-manual/time-sync.md).

## Q: Where is the API reference?
**A:** The Doxygen API reference is published under `api/doxygen/index.html` on GitHub Pages. See [API Reference](../api/doxygen/index.html).

## Q: What if my app cannot find shared libraries?
**A:** Set `LD_LIBRARY_PATH=~/tdk_install/lib` before running binaries.

## Q: Where can I get help?
**A:** Open an issue on GitHub or contact Flexiv via https://www.flexiv.com/contact.
