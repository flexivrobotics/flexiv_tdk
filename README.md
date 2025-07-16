# Flexiv TDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_tdk/actions/workflows/cmake.yml/badge.svg) ![Version](https://img.shields.io/badge/version-1.4-blue.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv TDK (Teleoperation Development Kit) is an SDK for developing customized robot-robot or device-robot teleoperation applications with Flexiv's adaptive robots. It features synchronized motions that are force-guided using high-fidelity perceptual feedback and supports both LAN (Local Area Network) and WAN (Wide Area Network, i.e. Internet) connections.

## Compatibility

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 20.04/22.04) | x86_64                  | C++                    | build-essential           |

## Quick Start

The TDK library is packed into a unified modern CMake project named ``flexiv_tdk``, which can be configured and installed using CMake on all supported OS.

### Install on Linux

1. In a new Terminal, install compiler kit, CMake (with GUI), Python interpreter, and Python package manager:

       sudo apt install build-essential cmake cmake-qt-gui -y

2. Choose a directory for installing ``flexiv_tdk`` library and all its dependencies. For example, a new folder named ``tdk_install`` under the home directory.

3. In a new Terminal, run the provided script to compile and install all dependencies to the installation directory chosen in step 2:

       cd flexiv_tdk/thirdparty
       bash build_and_install_dependencies.sh ~/tdk_install

   NOTE: Internet connection is required for this step.

4. In a new Terminal, configure ``flexiv_tdk`` library as a CMake project:

       cd flexiv_tdk
       mkdir build && cd build
       cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` is a CMake parameter specifying the path of the chosen installation directory. Alternatively, this configuration step can be done using CMake GUI.

5. Install ``flexiv_tdk`` library:

       cd flexiv_tdk/build
       cmake --build . --target install --config Release

   The library will be installed to ``CMAKE_INSTALL_PREFIX`` path, which may or may not be globally discoverable by CMake.

### Link to installed library from a user program

After the TDK library is installed, it can be found as a CMake target and linked to from other CMake projects. Using the provided examples project for instance::

    cd flexiv_tdk/example
    mkdir build && cd build
    cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
    cmake --build . --config Release -j 4

NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells the user project's CMake where to find the installed TDK library. The instruction above applies to all supported OS.

### Run example programs

To run a compiled example program:

    cd flexiv_tdk/example/build
    sudo ./<program_name> [arguments]

The exact arguments required by an example program are documented in the program's code. Also, ``sudo`` is required to grant root permissions.

## API Documentation

The API documentation can be generated using doxygen:

      sudo apt install doxygen-latex graphviz
      cd flexiv_tdk
      doxygen doc/Doxyfile.in

The generated API documentation is under ``flexiv_tdk/doc/html/`` directory. Open any html file with your browser to view it.
