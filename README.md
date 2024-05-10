# Flexiv Omni Teleop SDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_omni_teleop/actions/workflows/cmake.yml/badge.svg) ![Version](https://img.shields.io/badge/version-1.0-blue.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

The Flexiv Omni Teleop SDK provides C++ APIs for developing complex and customized applications involving robot-robot or device-robot teleoperation. The supported devices are listed below.

## Supported devices

| **Supported teleop type** | **Supported local machine** | **Supported remote machine** |
| ------------------------- | --------------------------- | ---------------------------- |
| robot-robot               | Rizon4s                     | Rizon4s                      |

## Compatibility

| **Supported OS**     | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 22.04) | x86_64                  | C++                    | build-essential           |

### Applying for Omni license
1. The Omni license is forcibly bound to the user's Ubuntu computer hardware, so before proceeding with the following steps, make sure you have prepared a physical computer with Ubuntu 22.04 and CPU platform x86_64. If you need compatible with other operating systems or CPU architectures, please contact Flexiv.
2. Clone the code and run the generator under ``flexiv_omni_teleop/omni_license_generator`` on your computer.
   ```bash
   ./generator
   ```
   This will generate a feature_id.txt. Send this file to Flexiv to apply for the Omni license.

   Note: The feature id is a unique identifier for your application and is bound to your computer. Please generate feature_id and use Omni license on the same computer.
3. After received the Omni license, extract the zip package to a safe directory. For example, a new folder named ``omni_license`` under the home directory.

### Compile and install for Linux

1. In a new Terminal, install C++ compiler, Git, and CMake (with GUI) using the package manager:

   ```bash
   sudo apt install build-essential libssl-dev git cmake cmake-qt-gui -y
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
2. Set all the robots to `Auto/Remote Mode` via flexiv Elements, then to run the compiled example program. to run an compiled example program:

   ./<program_name> [remote_robot_SN] [local_robot_SN] [path_to_omni_licenseCfg.json]

