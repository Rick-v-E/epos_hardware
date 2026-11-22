# Architecture — nexos_ros2_epos

## Overview

`nexos_ros2_epos` provides a **`ros2_control` System hardware interface**
(`hardware_interface::SystemInterface`, plugin `epos_hardware/EposHardware`)
that talks to one or more **maxon EPOS 2/4** motor controllers through maxon's
**EPOS Command Library** (`libEposCmd`).

```
┌────────────────────────┐   ros2_control    ┌───────────────────────────┐
│ controller_manager +   │  state/command    │ EposHardware (SystemIface) │
│ controllers (e.g.      │ ◀───interfaces───▶ │  per joint: pos/vel/current│
│ joint_state_broadcaster│                    └─────────────┬─────────────┘
│  + a position/velocity │                                  │ ctypes-free C++
│  controller)           │                      VCS_* calls │ (libEposCmd)
└────────────────────────┘                                  ▼
                                               ┌───────────────────────────┐
                                               │ EPOS Command Library       │
                                               │  (USB / "MAXON SERIAL V2") │
                                               └─────────────┬─────────────┘
                                                             │ FTDI D2XX (USB)
                                                             ▼
                                                   EPOS2 24/5  →  Motor
```

## Why not `ros2_canopen`?

The standard ROS 2 CANopen stack (`ros2_canopen`) requires **SocketCAN**. Our
EPOS2 is connected over **USB** via an FTDI chip with a **non-standard PID**
(VID `0403`, PID `a8b0`), driven by maxon's **D2XX** userspace driver — there is
**no `/dev/ttyUSB` and no CAN adapter**. `epos_hardware` wraps the EPOS Command
Library directly, which is exactly the right fit for this USB/D2XX setup.

## Interfaces

Per joint the driver exposes:

| Interface | Direction | Unit |
|-----------|-----------|------|
| `position` | state + command | rad |
| `velocity` | state + command | rad/s |
| `current`  | state + command | A |

The **active command interface** (position / velocity / current) is chosen at
runtime via `perform_command_mode_switch()`, which activates the corresponding
EPOS profile mode (`VCS_ActivateProfilePositionMode` / `…VelocityMode` /
`…CurrentMode`) and applies the configured profile limits.

> `position` and `velocity` are the standard `ros2_control` interface names.
> `current` is **driver-specific** (`epos_hardware::HW_IF_CURRENT`, see
> `include/epos_hardware/epos_hardware.hpp`) and therefore not picked up by
> `joint_state_broadcaster` on its own — add it through that controller's
> `map_interface_to_joint_state` parameter if you want it in `/joint_states`.

## Transmissions

Each joint must have a `transmission_interface/SimpleTransmission`. The driver
converts actuator ↔ joint via the transmission (mechanical reduction, sign). The
sign of the reduction sets the `reversed` flag.

## Units & conversions

- Position: encoder quad-counts ↔ rad (`VCS_GetPositionIs`, `VCS_MoveToPosition`),
  scaled by `encoder_resolution`.
- Velocity: rpm ↔ rad/s (`VCS_GetVelocityIs`, `VCS_MoveWithVelocity`).
- Current: mA ↔ A (`VCS_GetCurrentIs`, `VCS_SetCurrentMustEx`).

## Lifecycle

- `on_init` — parse URDF joint parameters + load transmissions.
- `on_configure` — enumerate EPOS devices, bind by serial number, clear faults.
- `on_activate` — enable the drive(s).
- `read` / `write` — per control cycle: read pos/vel/current, send command.
- `on_deactivate` — halt movement + disable.
- `on_cleanup` — close all devices.

## Build-time dependencies

`ros2_control` stack: `hardware_interface`, `controller_interface`,
`controller_manager`, `transmission_interface`, plus `pluginlib`,
`rclcpp_lifecycle`, `lifecycle_msgs`, `rclcpp`. The maxon `libEposCmd` /
`libftd2xx` are resolved **system-wide** (see [BUILD.md](BUILD.md)).
