# Flexiv TDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_tdk/actions/workflows/cmake.yml/badge.svg) ![Version](https://img.shields.io/badge/version-1.5-blue.svg) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv TDK (Teleoperation Development Kit) is an SDK for developing customized robot-robot or device-robot teleoperation applications with Flexiv's adaptive robots. It features synchronized motions that are force-guided using high-fidelity perceptual feedback and supports both LAN (Local Area Network) and WAN (Wide Area Network, i.e. Internet) connections.

### 🎬 TDK | Teleoperation Made Simple

[![Watch the video](https://img.youtube.com/vi/H0e9FSZIa14/hqdefault.jpg)](https://www.youtube.com/watch?v=H0e9FSZIa14)

## Compatibility

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit**                 | **Python interpreter** |
| -------------------------- | ----------------------- | ---------------------- | ----------------------------------------- | ---------------------- |
| Linux (Ubuntu 20.04/22.04) | x86_64, aarch64         | C++, python            | build-essential(GCC v9.4+, CMake 3.16.3+) | 3.8,3.10,3.12          |

Contact Flexiv for support if you need to run on other platforms.

## What Is the Low-Latency Kernel?

Ubuntu provides several kernel variants:
| Kernel Type       | Description                                                               | Typical Use Case                                        |
| ----------------- | ------------------------------------------------------------------------- | ------------------------------------------------------- |
| `generic`         | Default Ubuntu kernel with balanced performance and power management.     | Desktop, Server, General-purpose computing.             |
| `lowlatency`      | Preemptible kernel with reduced interrupt latency and tighter scheduling. | Audio processing, Robotics, Real-time simulations.      |
| `rt` (PREEMPT_RT) | Fully preemptible, real-time kernel with hard determinism.                | Industrial control, Mission-critical real-time systems. |

## Kernel setup for real-time performance

Using ``low-latency`` or ``PREEMPT_RT`` kernel can provider a better performance for robotics applications than the default ``generic`` kernel on Ubuntu platform. Using the ``PREEMPT_RT`` or ``low-latency`` kernel can achieve real-time performance, but it may encounter some known issues, especially those related to NVIDIA graphics card drivers. 

---

## ⚠️ Disclaimer

Upgrading your system to a **low-latency** or **real-time (RT) kernel** may introduce **driver compatibility issues** and could potentially **render your system unstable or unbootable**. Certain hardware drivers, proprietary modules, or third-party software may not fully support these kernel variants.  

By proceeding with any kernel upgrade, you **assume full responsibility** for any system malfunction, data loss, or hardware issues that may occur.  

**Please read and fully understand the risks** before attempting any installation, and ensure you have **proper backups** or recovery options in place. Proceed **with caution**.

---

## Install the Low-Latency or PREEMPT_RT Kernel on Ubuntu (Optional)

This guide explains the correct way to install and manage a **low-latency kernel** on Ubuntu systems, typically used for **robotics**, **real-time audio**, or **low-latency control** applications.

### Low-latency kernel

1. Install the low-latency kernel:
   
        sudo apt update && sudo apt install --install-recommends linux-lowlatency

    or if you are using the Hardware Enablement (HWE) kernel series:

       sudo apt update && sudo apt install linux-lowlatency-hwe-22.04

    where 22.04 should be replaced with your ubuntu version.

    #### 💡How do I determine if the -hwe suffix is ​​required?

    Run ``uname -r``. If the kernel version is higher than the base version (for example, the default for Ubuntu 22.04 is 5.15, but you're using 6.5), you're using the HWE kernel and should install the corresponding hwe low-latency package.


2. Boot order prefers low latency over generic kernel

        echo 'GRUB_FLAVOUR_ORDER="lowlatency"' | sudo tee -a /etc/default/grub

    Then update grub:

        sudo update-grub

3. Reboot the computer to boot into the low-latency kernel.
   
        uname -r 

    The output should be the version of lowlatency kernel like:

        5.15.0-145-lowlatency

NOTE: If you want to switch back to the generic kernel, just change ``lowlatency`` to ``generic``.

### PREEMPT_RT kernel
Following this tutorial: [Ubuntu20/22 RT kernel](https://www.flexiv.com/software/rdk/manual/realtime_ubuntu.html#ubuntu-22-04-24-04-enable-via-pro-subscription)

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


### System Clock Synchronization with Chrony (Required only for WAN Teleop)

1. Install ``chrony`` for synchronizing the system clock, this is required for teleoperation cross internet:

        sudo apt install chrony -y

    the chrony service will running automatically in the background.

2. Check the status of ``chrony`` service:

       systemctl status chrony

    The service will be running if you see the following output:

        Active: active (running)

3. ⏱️ Quick Check: Verify Current Time Synchronization Accuracy

    To quickly check how accurately your system clock is synchronized with the NTP servers:

        chronyc tracking | grep 'System time\|RMS offset'
    

    System time: the instantaneous offset between local system clock and NTP reference

    RMS offset: the long-term average offset (root mean square) over time

    Use the following table as a guideline:

    | Network Condition | Good (ms) | Acceptable (ms) | Poor (ms) |
    | ----------------- | --------- | --------------- | --------- |
    | System time       | < 1       | 1 - 10          | > 10      |
    | RMS offset        | < 5       | 5 - 20          | > 20      |

4. Improve Time Synchronization Accuracy Quickly
        
        sudo chronyc burst 4/4
        sudo chronyc makestep
    
    ``burst 4/4``: Immediately perform 4 quick communication rounds with the NTP server (by default, it usually waits for tens of seconds).
    
    ``makestep``: Immediately adjust the system clock to the correct time, instead of gradually correcting the drift.

    If the network environment changes (such as switching from Wi-Fi to Ethernet), it is recommended to restart the service:

        sudo systemctl restart chronyd
        sleep 5
        sudo chronyc makestep
5. To know more about ``chrony`` configuration for clock synchronization, please refer to the [Chrony](https://chrony-project.org/).

### Run example programs
To run a compiled example program:

    cd flexiv_tdk/example/build
    ./<program_name> [arguments]

The exact arguments required by an example program are documented in the program's code. Also, ``sudo`` is required to grant root permissions.

## API Documentation

The API documentation can be generated using doxygen:

      sudo apt install doxygen-latex graphviz
      cd flexiv_tdk
      doxygen doc/Doxyfile.in

The generated API documentation is under ``flexiv_tdk/doc/html/`` directory. Open any html file with your browser to view it.