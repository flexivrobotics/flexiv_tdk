# Flexiv TDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_tdk/actions/workflows/cmake.yml/badge.svg) ![Version](https://img.shields.io/badge/version-1.1-blue.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv TDK (Teleoperation Development Kit) is an SDK provides C++ APIs for developing complex and customized applications involving robot-robot or device-robot teleoperation. The supported devices are listed below.

## Supported devices

| **Supported teleop type** | **Supported local devices** | **Supported remote devices** |
| ------------------------- | --------------------------- | ---------------------------- |
| robot-robot               | Rizon4s                     | Rizon4s                      |

## Compatibility

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 20.04/22.04) | x86_64                  | C++                    | build-essential           |

### Install on Linux

1. In a new Terminal, install C++ compiler, libssl-dev, net-tools, Git, and CMake (with GUI) using the package manager:

       sudo apt install build-essential libssl-dev net-tools git cmake cmake-qt-gui -y

2. Choose a directory for installing ``flexiv_tdk`` library and all its dependencies. For example, a new folder named ``tdk_install`` under the home directory.

3. Please ensure that your network connection is unobstructed. Then, in a new Terminal, run the provided script to compile and install all dependencies to the installation directory chosen in step 2:

       cd flexiv_tdk/thirdparty
       bash build_and_install_dependencies.sh ~/tdk_install

4. In a new Terminal, use CMake to configure `flexiv_tdk`:

       cd flexiv_tdk
       mkdir build && cd build
       cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` is a CMake parameter specifying the path of the chosen installation directory. Alternatively, this configuration step can also be done through CMake GUI.

5. Install `flexiv_tdk` library:

       cd flexiv_tdk/build
       cmake --build . --target install --config Release

   NOTE: the installation of `flexiv_tdk` library is complete now. The following steps show how to link to the installed library from a user project.

### Link to the installed library to a user project

1. To find and link to the installed `flexiv_tdk` library to a user project, using the provided example project for instance:

       cd flexiv_tdk/example
       mkdir build && cd build
       cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install
       cmake --build . --config Release -j 4

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells user project's CMake where to find the installed `flexiv_tdk` library.

### Run tdk example

1. Apply for license to run tdk. See [apply_for_license.md](./license_generator/apply_for_license.md)

2. Set all the robots to `Auto/Remote Mode` via Flexiv Elements, then to run the compiled example program:

       ./<program_name> [local_robot_serial_number] [remote_robot_serial_number] [path_to_licenseCfg.json]


## API Documentation

API doc can be generated using doxygen

      sudo apt install doxygen-latex graphviz
      cd flexiv_tdk
      doxygen doc/Doxyfile.in

The generated API doc is under ```flexiv_tdk/doc/html ```. You can open any of the html file with your web browser to view it.
