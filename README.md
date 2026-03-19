# Flexiv TDK

[![CMake](https://github.com/flexivrobotics/flexiv_tdk/actions/workflows/cmake.yml/badge.svg)](https://github.com/flexivrobotics/flexiv_tdk/actions/workflows/cmake.yml)
[![Version](https://img.shields.io/badge/version-1.6-blue.svg)]()
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

**Flexiv TDK (Teleoperation Development Kit)** is an SDK for building custom robot-to-robot or device-to-robot teleoperation applications with Flexiv's adaptive robots. It enables synchronized, force-guided motion using **high-fidelity perceptual feedback** and supports both **LAN** (Local Area Network) and **WAN** (Internet) connections.

## Contact-Rich Manipulation Benchmarks
Flexiv TDK has been recognized on the [Manipulation Net Peg-in-Hole Leaderboard](https://manipulation-net.org/leaderboards/peg_in_hole.html). The [Manipulation Net](https://manipulation-net.org) is a public benchmark for robotic manipulation in the real world at scale with any robot at any time and anywhere. This provides external evidence for the contact-rich manipulation capability relevant to TDK use cases, including compliant insertion, alignment, and force-sensitive teleoperation workflows.


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

## References

[Flexiv TDK Page](https://flexivrobotics.github.io/flexiv_tdk/) is the main reference. It contains important information including user manual and API documentation. The instructions below serve as a quick reference, and you can find the full documentation at [Flexiv TDK Manual](https://flexivrobotics.github.io/flexiv_tdk/user-manual/overview/).

---

## ✅ Compatibility

| OS            | Processor       | Languages   | Compiler Requirements     | Python Versions |
| ------------- | --------------- | ----------- | ------------------------- | --------------- |
| Ubuntu 22.04+ | x86_64, aarch64 | C++, Python | GCC ≥ 9.4, CMake ≥ 3.16.3 | 3.8, 3.10, 3.12 |

>💡 Need support for other platforms? [Contact Flexiv](https://www.flexiv.com/contact).

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
LD_LIBRARY_PATH=~/tdk_install/lib ./<program_name> [arguments]
```
``LD_LIBRARY_PATH`` is used to specify where the shared libraries of the dependencies are installed.

Check each example’s source code for usage details.

## 📚 Generate API Documentation
The complete and detailed API documentation of the latest release can be found at [API Reference](https://flexivrobotics.github.io/flexiv_tdk/api/doxygen/index.html).
The API documentation of a previous release can be generated manually using Doxygen. For example, on Linux:
```bash
sudo apt install doxygen-latex graphviz
cd flexiv_tdk
git checkout <tag_name>
doxygen docs/doxygen/Doxyfile.in
```

Open flexiv_tdk/docs/api/doxygen/index.html in your browser.