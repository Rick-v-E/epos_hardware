# nexos_ros2_epos

ROS 2 `ros2_control` hardware interface for **maxon EPOS 2/4** motor controllers,
using the maxon **EPOS Command Library** over **USB** (FTDI D2XX).

NEXOS fork of [Rick-v-E/epos_hardware](https://github.com/Rick-v-E/epos_hardware)
(itself derived from [yoshito-n-students/eposx_hardware](https://github.com/yoshito-n-students/eposx_hardware)),
**ported to ROS 2 Lyrical** and with the proprietary maxon library **de-vendored**.

- Part of the NEXOS ROS 2 module family — see the repo/doc standard in
  [`nexos_ros2`](https://github.com/NEXOSGroup/nexos_ros2).
- License: **MIT** (upstream). The maxon EPOS Command Library is **proprietary and
  not included** — install it yourself (see [docs/BUILD.md](docs/BUILD.md)).

## Status

| Item | State |
|------|-------|
| Build on ROS 2 **Lyrical** / Ubuntu 26.04 (aarch64) | ✅ warning-free |
| Runtime (`list_available_devices`) | ✅ loads system `libEposCmd.so` |
| maxon library | de-vendored → installed system-wide |
| Real motor test | ⏳ pending (hardware) |

## Quickstart

```bash
# 1. Install ROS 2 Lyrical + ros2_control (see docs/BUILD.md)
sudo apt install ros-lyrical-ros-base ros-lyrical-ros2-control ros-dev-tools

# 2. Install the proprietary maxon EPOS Command Library system-wide
scripts/install_maxon_epos_library.sh /path/to/EPOS-Linux-Library-En.zip

# 3. Build
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone https://github.com/NEXOSGroup/nexos_ros2_epos.git src/nexos_ros2_epos
source /opt/ros/lyrical/setup.bash
colcon build --packages-select epos_hardware
source install/setup.bash

# 4. Find your controller's serial number
ros2 run epos_hardware list_available_devices
```

Then reference the `epos_hardware/EposHardware` plugin from your robot's URDF
`ros2_control` tag — see [docs/HARDWARE.md](docs/HARDWARE.md).

## Documentation

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — how the driver works
- [docs/BUILD.md](docs/BUILD.md) — prerequisites, library install, build, Lyrical port notes
- [docs/HARDWARE.md](docs/HARDWARE.md) — wiring, udev, URDF device parameters
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — common problems & fixes
- [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md) — open defects and traps, with what is proven and what is not

> Defence-specific deployment/security documentation is maintained privately in
> `nexos_ros2_defence` and is intentionally not part of this public repository.
