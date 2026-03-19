# Real-Time Kernel Options


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