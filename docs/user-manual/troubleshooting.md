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

## Leader robot drifting
- Check the inertial parameter calibration of the leader robot tool and confirm that the configured tool is indeed the current one.​
- Check that the TCP position of the leader robot is set at the actual position where the human hand grips it​
- When starting TDK examples, whether to calibrate the sensor is determined based on the actual input parameters. If choose to calibrate the sensor on initialization, the calibration will last for a period of time, so do not touch the device during that process.

## Getting Help
- [GitHub Issues](https://github.com/flexivrobotics/flexiv_tdk/issues)