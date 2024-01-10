# Flexiv Omni Teleop SDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_omni_teleop/actions/workflows/cmake.yml/badge.svg) ![Version](https://img.shields.io/badge/version-0.7-blue.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

The Flexiv Omni Teleop SDK provides C++ APIs for developing complex and customized applications involving robot-robot or device-robot teleoperation. The supported devices are listed below.

## Supported devices

| **Supported teleop type** | **Supported local machine** | **Supported remote machine** |
| ------------------------- | --------------------------- | ---------------------------- |
| robot-robot               | Rizon4s                     | Rizon4s                      |

## Compatibility

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 20.04/22.04) | x86_64                  | C++                    | build-essential           |

### Applying for a license
1. Run the generator to generate your feature ID and provide the feature id to apply for the license.
Note: The feature id is a unique identifier for your application and is bound to your local computer. Please generate feature ID and use license on the same computer.

### Compile and install for Linux

1. In a new Terminal, install C++ compiler, Git, and CMake (with GUI) using the package manager:

   ```bash
   sudo apt install build-essential git cmake cmake-qt-gui -y
   ```
2. Choose a directory for installing ``flexiv_omni_teleop`` library and all its dependencies. For example, a new folder named ``teleop_install`` under the home directory.
3. Please ensure that your network connection is unobstructed. Then, in a new Terminal, run the provided script to compile and install all dependencies to the installation directory chosen in step 2:

   ```bash
   cd flexiv_omni_teleop/thirdparty
   bash build_and_install_dependencies.sh ~/teleop_install
   ```
4. In a new Terminal, use CMake to configure `flexiv_omni_teleop`:

   ```bash
   cd flexiv_omni_teleop
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=~/teleop_install
   ```
   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` is a CMake parameter specifying the path of the chosen installation directory. Alternatively, this configuration step can also be done through CMake GUI.
5. Compile and install `flexiv_omni_teleop` library:

   ```bash
   cd flexiv_omni_teleop/build
   cmake --build . --target install --config Release
   ```
   NOTE: the installation of `flexiv_omni_teleop` library is complete now. The following steps show how to link to the installed library from a user project.
### Link to the installed library to a user project
1. To find and link to the installed `flexiv_omni_teleop` library to a user project, using the provided example project for instance:

   ```bash
   cd flexiv_omni_teleop/example
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=~/teleop_install
   cmake --build . --config Release -j 4
   ```
   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells user project's CMake where to find the installed `flexiv_omni_teleop` library.
2. Set all the robots were in `Auto/Remote Mode` via flexiv Elements, then to run an compiled example program:

   ```bash
   sudo ./<program_name> -l [local_robot_serial_number] -r [remote_robot_serial_number] -c [path-to-licenseCfg.json]
   ```
