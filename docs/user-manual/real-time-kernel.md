# Real-Time Kernel Options

Real-time performance can significantly improve teleoperation responsiveness. Ubuntu provides multiple kernel variants:

| Kernel Type         | Description                          | Typical Use Case                     |
| ------------------- | ------------------------------------ | ------------------------------------ |
| `generic`           | Default kernel; balanced performance | General desktop/server use           |
| `lowlatency`        | Reduced interrupt latency            | Robotics, audio, soft real-time      |
| `rt` (`PREEMPT_RT`) | Fully preemptible                    | Industrial control, mission-critical |

## Important Disclaimer
Upgrading to a low-latency or RT kernel **may**:
- Break proprietary drivers (e.g., NVIDIA, Wi-Fi)
- Cause instability or boot failure

**You assume full responsibility** for any issues arising from kernel changes. Always back up your system.

## Install Low-Latency Kernel (Ubuntu)

```bash
sudo apt update && sudo apt install --install-recommends linux-lowlatency-hwe-22.04  # replace 22.04 with your version
```

For the original 5.15 kernel:

```bash
sudo apt update && sudo apt install --install-recommends linux-lowlatency
```

Set GRUB to prefer low-latency:

```bash
echo 'GRUB_FLAVOUR_ORDER="lowlatency"' | sudo tee -a /etc/default/grub
sudo update-grub
```

Reboot and verify:

```bash
sudo reboot
uname -r  # should show "-lowlatency"
```

To revert to `generic`, update `GRUB_FLAVOUR_ORDER` and run `sudo update-grub`.

## PREEMPT_RT Kernel
Follow the official guide:
- https://www.flexiv.com/software/rdk/manual/realtime_ubuntu.html#ubuntu-22-04-24-04-enable-via-pro-subscription

> Ubuntu 22.04/24.04 users can enable RT kernel via **free Ubuntu Pro**. Ubuntu 20.04 requires manual patching.

> For NVIDIA Jetson (aarch64), refer to NVIDIA’s official documentation.
