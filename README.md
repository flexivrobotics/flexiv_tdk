# Flexiv Omni Teleop

![CMake Badge](https://github.com/flexivrobotics/flexiv_omni_teleop/actions/workflows/cmake.yml/badge.svg) ![Version](https://img.shields.io/badge/version-1.0-blue.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

The Flexiv Omni Teleop is an SDK provides C++ APIs for developing complex and customized applications involving robot-robot or device-robot teleoperation. The supported devices are listed below.

## Supported devices

| **Supported teleop type** | **Supported local machine** | **Supported remote machine** |
| ------------------------- | --------------------------- | ---------------------------- |
| robot-robot               | Rizon4s                     | Rizon4s                      |

## Compatibility

| **Supported OS**     | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 22.04) | x86_64                  | C++                    | build-essential           |

### Compile and install for Linux

1. In a new Terminal, install C++ compiler, libssl-dev, net-tools, Git, and CMake (with GUI) using the package manager:

      sudo apt install build-essential libssl-dev net-tools git cmake cmake-qt-gui -y

2. Choose a directory for installing ``flexiv_omni_teleop`` library and all its dependencies. For example, a new folder named ``teleop_install`` under the home directory.

3. Please ensure that your network connection is unobstructed. Then, in a new Terminal, run the provided script to compile and install all dependencies to the installation directory chosen in step 2:

      cd flexiv_omni_teleop/thirdparty
      bash build_and_install_dependencies.sh ~/teleop_install

4. In a new Terminal, use CMake to configure `flexiv_omni_teleop`:

      cd flexiv_omni_teleop
      mkdir build && cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=~/teleop_install

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` is a CMake parameter specifying the path of the chosen installation directory. Alternatively, this configuration step can also be done through CMake GUI.

5. Compile and install `flexiv_omni_teleop` library:

      cd flexiv_omni_teleop/build
      cmake --build . --target install --config Release

   NOTE: the installation of `flexiv_omni_teleop` library is complete now. The following steps show how to link to the installed library from a user project.

### Link to the installed library to a user project

1. To find and link to the installed `flexiv_omni_teleop` library to a user project, using the provided example project for instance:

      cd flexiv_omni_teleop/example
      mkdir build && cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=~/teleop_install
      cmake --build . --config Release -j 4

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells user project's CMake where to find the installed `flexiv_omni_teleop` library.

### Run Omni-Teleop

1. Apply for Omni license to run Omni-Teleop. See [Omni_license.md](omni_license_generator/Omni_license.md)

2. Set all the robots to `Auto/Remote Mode` via flexiv Elements, then to run the compiled example program:

      ./<program_name> [remote_robot_SN] [local_robot_SN] [path_to_omni_licenseCfg.json]

### API Documentation

API doc can be generated using doxygen

      sudo apt install doxygen-latex graphviz
      cd flexiv_omni_teleop
      doxygen doc/Doxygen.in

The generated API doc is under ```flexiv_omni_teleop/doc/html ```. You can open any of the html file with your web browser to view it.
