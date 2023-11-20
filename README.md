# flexiv_omni_teleoperation

![CMake](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg) ![Version](https://img.shields.io/badge/version-0.6-blue.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

The flexiv_omni_teleoperaion is a powerful development toolkit that enables the users to create complex and customized medical teleoperation related applications using APIs that provide reliable access to Flexiv robots.

## Compatibility

| **Supported OS**     | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------------- | ---------------------------- | ------------------------------- |
| Linux (Ubuntu 20.04/22.04) | x86_64, arm64                 | C++                          | build-essential                 |

### Compile and install for Linux

1. In a new Terminal, install C++ compiler, Git, and CMake (with GUI) using the package manager:
   ```bash
   sudo apt install build-essential git cmake cmake-qt-gui -y
   
   ```
2. Choose a directory for installing ``flexiv_omni_teleoperation`` library and all its dependencies. For example, a new folder named ``teleoperation_install`` under the home directory.
3. In a new Terminal, use CMake to configure `flexiv_omni_teleoperation`:
   ```bash
      cd flexiv_omni_teleoperation
      mkdir build && cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=~/teleoperation_install
   
   ```
   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` is a CMake parameter specifying the path of the chosen installation directory. Alternatively, this configuration step can also be done through CMake GUI.
4. Compile and install `flexiv_omni_teleoperation` library:
   ```bash   
      cd flexiv_omni_teleoperation/build
      cmake --build . --target install --config Release
   ```
   NOTE: the installation of `flexiv_omni_teleoperation` library is complete now. The following steps show how to link to the installed library from a user project.
5. To find and link to the installed `flexiv_omni_teleoperation` library from a user project, using the provided example project for instance:
   ```bash
      cd flexiv_omni_teleoperation/example
      mkdir build && cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=~/teleoperation_install
      cmake --build . --config Release -j 4
   ```
   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells user project's CMake where to find the installed `flexiv_omni_teleoperation` library.
6. Assuming all the robots were in `Auto/Remote Mode`, to run an compiled example program:
   ```bash
   sudo ./<program_name> -l [local_robot_serial_number] -r [remote_robot_serial_number] [...]
   ```
