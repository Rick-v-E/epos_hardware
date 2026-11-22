# Hardware & URDF — nexos_ros2_epos

## Controller & connection

- **Controller:** maxon **EPOS2 24/5** (also supports EPOS4), CiA 402 / CANopen
  profile, driven here over **USB**.
- **USB chip:** FTDI with a **non-standard PID** (VID `0403`, PID `a8b0`). The
  Linux kernel `ftdi_sio` driver does **not** bind it — access goes through
  maxon's **D2XX** userspace driver (`libftd2xx`). There is **no `/dev/ttyUSB`**.
- **udev / permissions:** maxon's installer adds `99-ftdi.rules` +
  `99-epos4.rules` and grants group access. If a device is not found, re-plug it
  and make sure your user is in the `plugdev`/`dialout` group (re-login after a
  group change).

## Finding the serial number

```bash
ros2 run epos_hardware list_available_devices
```
Prints every reachable controller with its **serial number** (hex), which you
put into the URDF below.

> ⚠️ This tool **segfaults when no controller is attached**, and the serial it
> reads may come from a different object than the identity object. Both are
> described, with a working alternative, in
> [KNOWN_ISSUES.md](KNOWN_ISSUES.md).
>
> If every query returns `0x34000007 "Serial data frame not received"` while
> the OS does see the device, the controller has no supply voltage — USB powers
> only its FTDI chip.

## URDF `ros2_control` block

```xml
<ros2_control name="epos" type="system">
  <hardware>
    <plugin>epos_hardware/EposHardware</plugin>
  </hardware>

  <joint name="pan_joint">
    <param name="device_name">EPOS2</param>
    <param name="protocol_stack_name">MAXON SERIAL V2</param>
    <param name="interface_name">USB</param>
    <param name="serial_number">0x00000000XXXXXXXX</param>
    <param name="clear_faults">true</param>
    <param name="encoder_resolution">512</param>
    <param name="profile_position_velocity">30</param>
    <param name="profile_position_acceleration">50</param>
    <param name="profile_position_deceleration">50</param>
    <param name="profile_velocity_acceleration">50</param>
    <param name="profile_velocity_deceleration">50</param>

    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <transmission name="pan_transmission">
    <plugin>transmission_interface/SimpleTransmission</plugin>
    <actuator name="pan_motor" role="actuator"/>
    <joint name="pan_joint" role="joint">
      <mechanical_reduction>16</mechanical_reduction>
    </joint>
  </transmission>
</ros2_control>
```

### Parameters

| Param | Required | Meaning |
|-------|----------|---------|
| `device_name` | yes | EPOS model string, e.g. `EPOS2` / `EPOS4` |
| `protocol_stack_name` | yes | `MAXON SERIAL V2` (USB) |
| `interface_name` | yes | `USB` |
| `serial_number` | yes | hex serial from `list_available_devices` |
| `encoder_resolution` | yes | encoder resolution — see the note below |
| `profile_position_velocity`, `profile_position_acceleration`, `profile_position_deceleration` | yes | profile limits for position mode |
| `profile_velocity_acceleration`, `profile_velocity_deceleration` | yes | profile limits for velocity mode |
| `clear_faults` | no (default `false`) | clear EPOS faults on configure |
| `use_dummy` | no (default `false`) | simulate this joint without hardware — see [Dummy mode](#dummy-mode) |
| `mechanical_reduction` | yes | gearbox ratio in the transmission (sign = direction) |

A missing **required** parameter aborts `on_init` with an error naming the
parameter. The two optional flags are parsed as booleans that default to `false`
when absent; only the exact string `true` enables them.

### `encoder_resolution`

`read()` converts the raw position with:

```cpp
actuator.state.position = position_raw * M_PI_2 / encoder_resolution;   // [rad]
```

`VCS_GetPositionIs` returns **quad-counts**, and `M_PI_2` is **π/2**. So the
parameter has to be the encoder's **lines (counts per turn, CPT)** — *not* the
quad-count total — for one full turn to come out as 2π:

| Encoder | `encoder_resolution` | one turn reads as |
|---------|----------------------|-------------------|
| 512 CPT (2048 qc/turn) | `512` | 2π ✅ |
| 512 CPT (2048 qc/turn) | `2048` | π/2 ❌ (4× too small) |

Setting the quad-count total here scales every position reading **4× too small**.
Verify on the real axis once: command one full output revolution and check that
the reported joint position changes by 2π × `mechanical_reduction`.

## Dummy mode

With `use_dummy=true` the driver never touches hardware for that joint: device
enumeration is skipped in `on_configure`, no `VCS_*` call is issued, and `write()`
returns without sending anything. Instead `read()` feeds the command back as
state — in position mode the state position follows the command directly, in
velocity mode the position is integrated from `command.velocity × period`
(wrapped to 2π).

The flag is **per joint**, so dummy and real joints can be mixed in one hardware
component. This makes it possible to bring up and test the whole ROS 2 side —
controllers, teleop, limits, launch files — with **no EPOS hardware attached**.

## Reference test motor (NEXOS PTU25M)

- Motor: maxon **RE-max** (maxon no. 279980), brushed, GPX32 gearbox **16:1**,
  encoder **512 CPT** (2048 qc/rev) → `encoder_resolution` = **512**.
- EPOS2 24/5: HW `0x6220`, FW `0x2123`.

> ⚠️ Any `write()`/activation **physically moves the motor**. Ensure the axis is
> free and cleared with the operator before enabling.
