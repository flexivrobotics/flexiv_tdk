# Flexiv TDK

[![CMake](https://github.com/flexivrobotics/flexiv_tdk/actions/workflows/cmake.yml/badge.svg)](https://github.com/flexivrobotics/flexiv_tdk/actions/workflows/cmake.yml)
[![Version](https://img.shields.io/badge/version-1.5-blue.svg)]()
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

**Flexiv TDK (Teleoperation Development Kit)** is an SDK for building custom robot-to-robot or device-to-robot teleoperation applications with Flexiv's adaptive robots. It enables synchronized, force-guided motion using **high-fidelity perceptual feedback** and supports both **LAN** (Local Area Network) and **WAN** (Internet) connections.

🎬 **[Flexiv's TDK | Teleoperation Made Simple](https://www.youtube.com/watch?v=H0e9FSZIa14)**  
*(Click image below to play)*  
<p align="center">
  <a href="https://www.youtube.com/watch?v=H0e9FSZIa14" target="_blank">
    <img src="https://img.youtube.com/vi/H0e9FSZIa14/hqdefault.jpg" alt="TDK Demo 1" width="350" style="margin-right:10px;" />
  </a>
  <a href="https://www.youtube.com/watch?v=udkddqxth5Q" target="_blank">
    <img src="https://img.youtube.com/vi/udkddqxth5Q/hqdefault.jpg" alt="TDK Demo 2" width="350" />
  </a>
</p>


---

## ✅ Compatibility

| OS            | Processor       | Languages   | Compiler Requirements     | Python Versions |
| ------------- | --------------- | ----------- | ------------------------- | --------------- |
| Ubuntu 22.04+ | x86_64, aarch64 | C++, Python | GCC ≥ 9.4, CMake ≥ 3.16.3 | 3.8, 3.10, 3.12 |

>💡 Need support for other platforms? [Contact Flexiv](https://www.flexiv.com/contact).

---

## ⚙️ Kernel Options for Real-Time Performance

Ubuntu offers multiple kernel variants tailored for different workloads:

| Kernel Type         | Description                                                 | Typical Use Case                             |
| ------------------- | ----------------------------------------------------------- | -------------------------------------------- |
| `generic`           | Default kernel: balanced performance & power management     | General desktop/server use                   |
| `lowlatency`        | Reduced interrupt latency; better scheduling responsiveness | Robotics, audio processing, soft real-time   |
| `rt` (`PREEMPT_RT`) | Fully preemptible; hard real-time determinism               | Industrial control, mission-critical systems |

---

## ⚠️ Important Disclaimer

Upgrading to a **low-latency** or **real-time (RT) kernel** may:
- Break proprietary drivers (e.g., NVIDIA, Wi-Fi modules)
- Cause system instability or boot failure

**You assume full responsibility** for any issues arising from kernel changes.  
✅ **Always back up your system** before proceeding.

---


## Install Low-Latency or PREEMPT_RT Kernel for Ubuntu/x84-64 

### Option 1: Low-Latency Kernel

1. **Install the kernel**:
   For Hardware Enablement (HWE) stack (check with `uname -r`; e.g., Ubuntu 22.04 with kernel 6.x), use:

   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency-hwe-22.04  # replace "22.04" with your version
   ```
   For the original kernel (5.15), use:
   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency
   ```

2. **Set GRUB to prefer low-latency**:
   ```bash
    echo 'GRUB_FLAVOUR_ORDER="lowlatency"' | sudo tee -a /etc/default/grub
    sudo update-grub
   ```

3. **Reboot and verify**:
    ```bash
    sudo reboot
    uname -r  # Should show "...-lowlatency"
    ```
    🔄 To revert to ``generic``, change ``GRUB_FLAVOUR_ORDER="generic"`` and run ``sudo update-grub``.

### Option 2: PREEMPT_RT Kernel

Follow the official guide:  
🔗 [Real-time Ubuntu Setup (Documentation)](https://www.flexiv.com/software/rdk/manual/realtime_ubuntu.html#ubuntu-22-04-24-04-enable-via-pro-subscription)

> ℹ️ Ubuntu 22.04/24.04 users can enable RT kernel via **free Ubuntu Pro subscription**.  
> Ubuntu 20.04 requires manual patching (advanced users only).

> ℹ️ For  Nvidia Jetson(**arrch64**), please refer to Nvidia's official documentation.
---

## 🚀 Quick Start - Python

### 1. Install the Python package

On all supported platforms, the Python package of TDK and its dependencies for a specific Python version can be installed using the `pip` module:

    python3.x -m pip install spdlog flexivtdk

NOTE: replace `3.x` with a specific Python version.

### 2. Use the installed Python package

After the ``flexivtdk`` Python package is installed, it can be imported from any Python script. Test with the following commands in a new Terminal, which should start Flexiv TDK:

    python3.x
    import flexivtdk
    flexivtdk.__version__ 

### 3.🕒 System Clock Sync 

Accurate time sync is critical for teleop over the internet. For WAN teleop, there are two edge computers, one acting as a server and the other as a client, and the system time of these two computers needs to be calibrated. 

Note: This is only required for **WAN** Teleoperation, users can skip this section if only using LAN teleoperation.

1. Install & Start chrony
```bash
sudo apt install chrony -y
systemctl status chrony  # Should show "active (running)"
```
2. Check Sync Accuracy
```bash
chronyc tracking | grep 'System time\|RMS offset'
```

System time: the instantaneous offset between local system clock and NTP reference

RMS offset: the long-term average offset (root mean square) over time

| Network Condition | Good (ms) | Acceptable (ms) | Poor (ms) |
| ----------------- | --------- | --------------- | --------- |
| System time       | < 1       | 1 - 10          | > 10      |
| RMS offset        | < 5       | 5 - 20          | > 20      |


3. Force Immediate Sync (if needed)
```bash
sudo chronyc burst 4/4
sudo chronyc makestep
```
🔄 After network changes (e.g., Wi-Fi → Ethernet), restart:

```bash
sudo systemctl restart chronyd
sleep 5
sudo chronyc makestep
```
4. Learn more: [Chrony Documentation](https://chrony-project.org/)

### 4. Enable Real-Time Privileges for Non-root Users

To allow a regular user to create high-priority (real-time) threads without `sudo`, configure system to apply real-time and nice priority limits:

```bash
echo "${USER}    -   rtprio    99" | sudo tee -a /etc/security/limits.conf
echo "${USER}    -   nice     -20" | sudo tee -a /etc/security/limits.conf
```
Log out and log back in (or reboot) for the settings to take effect.

### 5. Run example Python scripts

To run an example Python script in this repo:

```bash
cd flexiv_tdk/example_py
python3.x <example_name>.py [arguments]
```

Check each example’s source code for usage details.

## 🚀 Quick Start - C++

The TDK is distributed as a modern CMake project named `flexiv_tdk`.

### 1. Install Build Dependencies
```bash
sudo apt install build-essential cmake cmake-qt-gui -y
```

### 2. Choose an Installation Directory

Example: `~/tdk_install`

### 3. Build & Install Third-Party Dependencies
```bash
cd flexiv_tdk/thirdparty
bash build_and_install_dependencies.sh ~/tdk_install
```

### 4. Configure & Install TDK
```bash
cd flexiv_tdk
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/tdk_install
cmake --build . --target install --config Release
```
> 🌐 Internet connection to GitHub is required for  [3. Build & Install Third-Party Dependencies](#3-build--install-third-party-dependencies) and [4. Configure & Install TDK](#4-configure--install-tdk).

### 5. Link TDK in Your Project

After the TDK library is installed, it can be found as a CMake target and linked to from other CMake projects. Using the provided examples project for instance:

```bash
cd flexiv_tdk/example
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=~/tdk_install
cmake --build . --config Release -j 4
```

NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells the user project's CMake where to find the installed TDK library. 



### 6. Run Examples
```bash
cd flexiv_tdk/example/build
./<program_name> [arguments]
```

Check each example’s source code for usage details.

## 📚 Generate API Documentation

```bash
sudo apt install doxygen-latex graphviz
cd flexiv_tdk
doxygen doc/Doxyfile.in
```

Open flexiv_tdk/doc/html/index.html in your browser.