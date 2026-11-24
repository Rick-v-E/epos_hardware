# Known issues

Things that bite in practice, with what is actually established and what is
still only suspected. Anything unproven says so.

## `list_available_devices` segfaults with no controller attached

**Status:** reproducible, cause not localised.

`ros2 run epos_hardware list_available_devices` terminates with a segmentation
fault (exit 139) when no EPOS controller is present:

```console
$ ros2 run epos_hardware list_available_devices
[ros2run]: Segmentation fault
```

No output appears before the crash, so it happens during enumeration rather
than while printing the table. The expected behaviour would be the header row
followed by `No devices found!`.

Observed on ROS 2 Lyrical / Ubuntu 26.04 / x86_64, built at `v0.1.0`, against
EPOS Command Library 6.8.1.0 (`lib/intel/x86_64`).

`check_single_device` crashes in its own empty case too, but for a different
and now localised reason - see below. The claim that it handles that case
cleanly stood here until 2026-11-25 and was wrong.

**What it is not.** The obvious theory, an unchecked `CreateDeviceHandle`, does
not hold: `EposFactory::CreateDeviceHandle` returns an empty `DeviceHandlePtr`
when `VCS_OpenDevice` fails (`src/utils.cpp:184-185`), and
`EposFactory::EnumerateNodes` checks that and returns early
(`src/utils.cpp:232-236`). The fault is more likely inside the maxon
enumeration helpers on the path where there is nothing to enumerate. No
debugger was available on the machine where this was found, so the
investigation stopped there rather than reaching a conclusion.

**Workaround.** Query the controller directly; this needs no ROS and no build.
Open `EPOS2 / MAXON SERIAL V2 / USB / USB0`, scan node ids 1..8 with
`VCS_GetVersion`, then read the identity object:

```python
# ctypes, EposCmd64.dll on Windows or libEposCmd.so on Linux
serial, read = c_uint(), c_uint()
lib.VCS_GetObject(handle, node, c_ushort(0x1018), c_ubyte(4),
                  byref(serial), c_uint(4), byref(read), byref(err))
```

This produced serial numbers matching the labels on two EPOS2 24/5 units
(HW `0x6220`, FW `0x2123` and `0x2126`).

## Serial number object location

**Status:** suspicion, unverified.

For `EPOS2`, `EnumerateNodes` reads the serial number from object `0x2004`
sub `0x00` (`src/utils.cpp:252`).

On the EPOS2 24/5 units above, the serial number was readable from `0x1018`
sub `4`, the CiA 301 identity object, and matched the physical labels. What
`0x2004` returns on those units has not been checked, so this may simply be a
second, EPOS2-specific location. Worth confirming before relying on the serial
that `list_available_devices` prints.

## `0x34000007 "Serial data frame not received"` means the controller has no supply

**Status:** confirmed, not a defect.

A USB cable powers only the FTDI chip in the controller. The operating system
therefore enumerates the device - on Windows it even shows up as
`maxon motor EPOS` under VID `0403` / PID `a8b0` - while every query fails:

```
VCS_OpenDevice   -> succeeds
VCS_GetProtocolStackSettings -> succeeds (1000000 baud, 500 ms)
VCS_GetVersion, every node id 1..8 -> 0x34000007 Serial data frame not received
```

The controller logic needs its own supply on the power input (11-24 V for the
EPOS2 24/5). Connect it and the same queries answer immediately. The error
does not indicate a broken controller or a wrong node id.

## maxon's `install.sh` writes a sudoers file

**Status:** confirmed, affects anyone installing the library as root.

Besides copying the library and installing udev rules, maxon's own
`install.sh` does:

```bash
touch -f /etc/sudoers.d/mmc_rule
echo $SUDO_USER 'ALL=(ALL) NOPASSWD: /bin/ip' > /etc/sudoers.d/mmc_rule
```

If the script runs as root **without** `sudo` - a plain root shell, a
container, `wsl -u root` - then `$SUDO_USER` is empty and the resulting line
has no user name. A malformed file in `/etc/sudoers.d/` can make `sudo` refuse
to work at all.

Run the installer through `sudo` so `$SUDO_USER` is set. On a development
machine that only ever uses `use_dummy`, the udev rules and the sudoers entry
serve no purpose and the library can simply be copied by hand:

```bash
# x86_64 example; use lib/arm/v8 on arm64
install -d /opt/EposCmdLib_6.8.1.0/lib
cp -r include /opt/EposCmdLib_6.8.1.0/
cp -r lib/intel/x86_64 /opt/EposCmdLib_6.8.1.0/lib/x86_64
chmod -R a+rX /opt/EposCmdLib_6.8.1.0
ln -sf /opt/EposCmdLib_6.8.1.0/lib/x86_64/libEposCmd.so.6.8.1.0 /usr/lib/libEposCmd.so
ln -sf /opt/EposCmdLib_6.8.1.0/lib/x86_64/libftd2xx.so.1.4.8     /usr/lib/libftd2xx.so
ldconfig
```

The `chmod` is not optional: the archive carries its library directories as
`drwx------`, and without it the later `dlopen` fails with
`Permission denied`.

## `check_single_device` is not a read-only check

**Status:** confirmed by reading the source and by running it.

The name reads like an inspection. It is not. After querying the controller it
**enables the drive and commands motion** (`src/check_single_device.cpp:100-116`):

```cpp
std::cout << "Enabling Motor" << std::endl;
VCS_WRAPPER_NA(VCS_SetEnableState, node_handle_);
VCS_WRAPPER_NA(VCS_ActivateProfileVelocityMode, node_handle_);

std::cout << "Start moving Motor for 10 seconds" << std::endl;
VCS_WRAPPER(VCS_MoveWithVelocity, node_handle_, 1000.0);
```

The commanded value is **1000 rpm at the motor, for ten seconds**, in Profile
Velocity Mode. The `Moving with velocity of ...` lines that follow are the
*measured* velocity (`VCS_GetVelocityIs`, `:110`), not the command - with no
motor attached they read `0`, which makes the output look harmless when it is
not.

It also **clears faults** rather than only reporting them (`:61-67`): if
`VCS_GetNbOfDeviceError` returns anything, it calls `VCS_ClearFault`. A fault
you wanted to keep for diagnosis is gone afterwards.

In full, the tool disables the drive, reads the error list, clears it if
non-empty, reads the velocity and position profiles, enables the drive, runs
the ten-second velocity move, halts, and disables again.

**Do not run it on an axis that must not move.** On the NEXOS PTU25M this is a
hardware hazard rather than an inconvenience: the tilt cable is routed through
the pan axis, there is no slip ring, and neither axis has end stops, so an
unattended ten-second move can tear the cable. Use it only with the output
decoupled or the axis cleared, and know where the supply switch is.

For an actual read-only identity check there is no tool in this package. Query
the controller directly instead - the `VCS_GetObject` snippet under
*`list_available_devices` segfaults* above reads the serial number and touches
nothing else.

## `check_single_device` dereferences a null handle when the serial is not found

**Status:** confirmed, cause localised, fix obvious.

`src/check_single_device.cpp:38-43`:

```cpp
if (!node_handle_)
{
    std::cout << "Could not find motor" << std::endl;
}

std::cout << "Found motor " << argv[1] << std::endl;
```

There is no `return` in that branch. The program reports the failure, then
immediately reports success, then enters the `try` block and passes the null
handle to `VCS_SetDisableState` (`:47`):

```console
$ ros2 run epos_hardware check_single_device deadbeef
Searching for motor deadbeef...
FT_DEVICE_NOT_FOUND:FT_OpenEx: Success
Could not find motor
Found motor deadbeef
[ros2run]: Segmentation fault
```

Observed on ROS 2 Lyrical / Ubuntu 26.04 / arm64 (Raspberry Pi 4), built at
`v0.1.0`, against EPOS Command Library 6.8.1.0 (`lib/arm/v8`), with two EPOS2
24/5 units attached and powered - the crash is about the *queried* serial not
matching, not about the bus being empty.

A `return 1;` inside the branch fixes it. Until then: the `Found motor` line
is not evidence that a controller was found. Read the line above it.

This is a different defect from the `list_available_devices` segfault further
up, which is still unlocalised.
