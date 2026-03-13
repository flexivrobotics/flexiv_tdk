# Troubleshooting

## Build Issues
- **CMake cannot find Flexiv TDK**: Ensure `-DCMAKE_PREFIX_PATH=~/tdk_install` points to the install prefix.
- **Missing dependencies**: Re-run `thirdparty/build_and_install_dependencies.sh` and ensure the install directory exists.

## Runtime Issues
- **Shared library not found**: Set `LD_LIBRARY_PATH=~/tdk_install/lib` before running binaries.
- **Permission issues with real-time priorities**: Add your user to `/etc/security/limits.conf` for `rtprio` and `nice`.

## WAN Teleoperation
- **High latency / unstable motion**: Verify network stability and re-check time synchronization.
- **Time sync inaccurate**: Use `chronyc tracking` and consider re-running `chronyc burst` and `chronyc makestep`.

## Getting Help
- [GitHub Issues](https://github.com/flexivrobotics/flexiv_tdk/issues)
