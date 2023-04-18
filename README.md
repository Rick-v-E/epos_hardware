# Maxon Epos Hardware Interface
ROS2 control hardware interface for Maxon Epos 2/4 motor controllers using the EPOS Command Library 6.8.1.0.

Adapted from https://github.com/yoshito-n-students/eposx_hardware and converted to ROS2.

## Installation
1. Install UDEV rules:
```commandline
ros2 run epos_hardware install_udev_rules
```
2. Get motor controllers serial id:
```commandline
ros2 run epos_hardware list_available_devices
```

## Usage
Add the plugin to the `ros2_control` tag in your URDF file:
```xml
<ros2_control name="epos_hardware" type="system">
    <hardware>
        <plugin>epos_hardware/EposHardware</plugin>

         <joint name="front_left_wheel_joint">
            <param name="device_name">EPOS2</param>
            <param name="protocol_stack_name">MAXON SERIAL V2</param>
            <param name="interface_name">USB</param>
            <param name="serial_number">0xxxxxxxxxxxxx</param>
            <param name="clear_faults">false</param>
            <param name="profile_acceleration">50</param>
            <param name="profile_deceleration">50</param>

            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <transmission name="rear_right_wheel_joint_transmission">
            <plugin>transmission_interface/SimpleTransmission</plugin>
            <actuator name="rear_right_motor" role="rear_right_motor"/>
            <joint name="rear_right_wheel_joint" role="rear_right_wheel_joint">
                <mechanical_reduction>53</mechanical_reduction>
            </joint>
        </transmission>
        
        <joint name="front_right_wheel_joint">
            <param name="device_name">EPOS2</param>
            <param name="protocol_stack_name">MAXON SERIAL V2</param>
            <param name="interface_name">USB</param>
            <param name="serial_number">0xxxxxxxxxxxxx</param>
            <param name="clear_faults">false</param>
            <param name="profile_acceleration">50</param>
            <param name="profile_deceleration">50</param>

            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
         <transmission name="front_right_wheel_joint_transmission">
            <plugin>transmission_interface/SimpleTransmission</plugin>
            <actuator name="rear_right_motor" role="rear_right_motor"/>
            <joint name="front_right_wheel_joint" role="front_right_wheel_joint">
                <mechanical_reduction>-53</mechanical_reduction>
            </joint>
        </transmission>
</ros2_control>
```

In our setup, every controller is connected seperately to the computer using USB, however, all methods that work with the Epos Command Library should also work with this code. For each joint, an `device_name`, `protocol_stack_name`, `interface_name`, `serial_number` and `encoder_resolution` parameter should be set. The `profile_acceleration` and `profile_deceleration` parameters define the acceleration and deceleration of the motors in rad/s<sup>2</sup>. Additionally, a `clear_faults` parameter can be set, which clears all device faults upon start.

## Supported command interfaces
Command interface `position`, `velocity` and `current` are supported.

## Supported state interfaces
The following state interfaces are supported:

* `position` (Position)
* `velocity` (Velocity) 
* `current` (Current \[A\])
