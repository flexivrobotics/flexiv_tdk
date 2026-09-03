# Q&A

## Q: Which platforms are supported?
**A:** Ubuntu 22.04+ on x86_64 and aarch64. C++ and Python are supported with GCC ≥ 9.4 and CMake ≥ 3.16.3. TDK v1.6.3 requires flexiv_rdk 1.9.3 and zenoh 1.9.0, and is compatible with Flexiv software package v3.11.2.

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
