/*
Copyright (c) 2023 Rick-v-E. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#include "epos_hardware/epos_hardware.hpp"

namespace epos_hardware
{
    static const std::string HW_NAME = "EposHardware";

    CallbackReturn EposHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        // Custom bind on_shutdown to stop motors when shutting down control mode.
        // TODO: remove once https://github.com/ros-controls/ros2_control/issues/472 is solved
        rclcpp::on_shutdown(std::bind(&EposHardware::stop, this));

        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

        actuators_.resize(info_.joints.size(), Actuator());

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            actuators_[i].name = info_.joints[i].name;

            if (getParamString(info_.joints[i].parameters, "device_name", &actuators_[i].device_name) != return_type::OK)
            {
                return CallbackReturn::ERROR;
            }

            if (getParamString(info_.joints[i].parameters, "protocol_stack_name", &actuators_[i].protocol_stack_name) != return_type::OK)
            {
                return CallbackReturn::ERROR;
            }

            if (getParamString(info_.joints[i].parameters, "interface_name", &actuators_[i].interface_name) != return_type::OK)
            {
                return CallbackReturn::ERROR;
            }

            std::string serial_number_hex;
            if (getParamString(info_.joints[i].parameters, "serial_number", &serial_number_hex) != return_type::OK)
            {
                return CallbackReturn::ERROR;
            }
            SerialNumberFromHex(serial_number_hex, &actuators_[i].serial_number);

            if (getParamBoolean(info_.joints[i].parameters, "clear_faults", &actuators_[i].clear_faults) != return_type::OK)
            {
                return CallbackReturn::ERROR;
            }

            if (getParamInt(info_.joints[i].parameters, "encoder_resolution", &actuators_[i].encoder_resolution) != return_type::OK)
            {
                return CallbackReturn::ERROR;
            }

            // Find transmission corresponding with joint
            for (auto transmission_info : info_.transmissions)
            {
                if (transmission_info.joints.size() != 1)
                {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(HW_NAME), "Ignoring transmission '" << transmission_info.name << "' because it has more than one joint!");
                    continue;
                }

                if (transmission_info.joints[0].name == info_.joints[i].name)
                {
                    if (transmission_info.type != "transmission_interface/SimpleTransmission")
                    {
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Transmission '" << transmission_info.name << "' of type '" << transmission_info.type << "' is not supported!");
                        return CallbackReturn::ERROR;
                    }

                    std::shared_ptr<transmission_interface::Transmission> transmission;
                    try
                    {
                        transmission = transmission_loader.load(transmission_info);
                    }
                    catch (const transmission_interface::TransmissionInterfaceException &exc)
                    {
                        RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Error while loading '" << transmission_info.name << "': " << exc.what());
                        return hardware_interface::CallbackReturn::ERROR;
                    }

                    std::vector<transmission_interface::JointHandle> joint_handles;
                    for (const auto &joint_info : transmission_info.joints)
                    {
                        transmission_interface::JointHandle joint_handle_position(joint_info.name, hardware_interface::HW_IF_POSITION, &actuators_[i].joint.transmission_passthrough_.position);
                        transmission_interface::JointHandle joint_handle_velocity(joint_info.name, hardware_interface::HW_IF_VELOCITY, &actuators_[i].joint.transmission_passthrough_.velocity);

                        joint_handles.push_back(joint_handle_position);
                        joint_handles.push_back(joint_handle_velocity);
                    }

                    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
                    for (const auto &actuator_info : transmission_info.actuators)
                    {
                        transmission_interface::ActuatorHandle actuator_handle_position(actuator_info.name, hardware_interface::HW_IF_POSITION, &actuators_[i].actuator.transmission_passthrough_.position);
                        transmission_interface::ActuatorHandle actuator_handle_velocity(actuator_info.name, hardware_interface::HW_IF_VELOCITY, &actuators_[i].actuator.transmission_passthrough_.velocity);

                        actuator_handles.push_back(actuator_handle_position);
                        actuator_handles.push_back(actuator_handle_velocity);
                    }

                    try
                    {
                        transmission->configure(joint_handles, actuator_handles);
                    }
                    catch (const transmission_interface::TransmissionInterfaceException &exc)
                    {
                        RCLCPP_FATAL_STREAM(rclcpp::get_logger(HW_NAME), "Error while configuring '" << transmission_info.name << "': " << exc.what());
                        return hardware_interface::CallbackReturn::ERROR;
                    }

                    actuators_[i].reversed = std::signbit(transmission_info.joints[0].mechanical_reduction);
                    actuators_[i].transmission = transmission;

                    RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Loaded transmission '" << transmission_info.name << "' for joint '" << info_.joints[i].name << "'");
                }
            }

            if (actuators_[i].transmission == nullptr)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Could find transmission for joint '" << info_.joints[i].name << "'");
                return CallbackReturn::ERROR;
            }
        }

        if (getParamBoolean(info_.hardware_parameters, "use_dummy", &use_dummy_) != return_type::OK)
        {
            return CallbackReturn::ERROR;
        }

        // Define dummy mode to test hardware interface without hardware connected
        if (use_dummy_)
        {
            RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "dummy mode");
            return CallbackReturn::SUCCESS;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EposHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (use_dummy_)
        {
            return CallbackReturn::SUCCESS;
        }

        // Store discovered EPOS devices in a map, so that we don't have to search for EPOS devices
        // when we have the same device_name, protocol stack name and interface name.
        std::map<std::tuple<std::string, std::string, std::string>, std::vector<EnumeratedNode>> discoveredDevicesMap;

        for (size_t i = 0; i < actuators_.size(); i++)
        {
            try
            {
                unsigned int error_code = 0;

                auto key = std::make_tuple(actuators_[i].device_name, actuators_[i].protocol_stack_name, actuators_[i].interface_name);
                auto it1 = discoveredDevicesMap.find(key);

                std::vector<EnumeratedNode> nodes;
                if (it1 != discoveredDevicesMap.end())
                {
                    nodes = it1->second;
                }
                else
                {
                    epos_factory_.EnumerateNodes(
                        actuators_[i].device_name,
                        actuators_[i].protocol_stack_name,
                        actuators_[i].interface_name,
                        &nodes,
                        &error_code);

                    discoveredDevicesMap.insert(std::make_pair(key, nodes));
                }

                actuators_[i].node_handle = epos_factory_.CreateNodeHandle(
                    nodes,
                    actuators_[i].serial_number,
                    &error_code);

                if (actuators_[i].node_handle == nullptr)
                {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Could not find motor with serial '0x" << actuators_[i].serial_number << "' for joint '" << actuators_[i].name << "'!");
                    return CallbackReturn::ERROR;
                }

                RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Connected to motor controller '0x" << actuators_[i].serial_number << "' for joint '" << actuators_[i].name << "'");

                // Disable controller
                VCS_WRAPPER_NA(VCS_SetDisableState, actuators_[i].node_handle);

                if (actuators_[i].clear_faults)
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Clearing faults for joint '" << actuators_[i].name << "'");

                    if (clearDeviceErrors(actuators_[i].node_handle) != return_type::OK)
                    {
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Cannot clear faults for joint '" << actuators_[i].name << "'");
                    }
                }
            }
            catch (const EposException &error)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), error.what());
                return CallbackReturn::ERROR;
            }
        }

        // Init command modes
        std::vector<std::string> new_interfaces, old_interfaces;
        for (auto joint : info_.joints)
        {
            for (auto interface : joint.command_interfaces)
            {
                new_interfaces.push_back(joint.name + "/" + interface.name);
            }
        }
        perform_command_mode_switch(new_interfaces, old_interfaces);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EposHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (use_dummy_)
        {
            return CallbackReturn::SUCCESS;
        }

        unsigned int error_code = 0;
        if (VCS_CloseAllDevices(&error_code))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Could not close all motor controllers: " << EposException::toErrorInfo(error_code));
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> EposHardware::export_state_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "export_state_interfaces");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < actuators_.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                actuators_[i].name, hardware_interface::HW_IF_POSITION, &actuators_[i].joint.state.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                actuators_[i].name, hardware_interface::HW_IF_VELOCITY, &actuators_[i].joint.state.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                actuators_[i].name, epos_hardware::HW_IF_CURRENT, &actuators_[i].joint.state.current));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> EposHardware::export_command_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "export_command_interfaces");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < actuators_.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                actuators_[i].name, hardware_interface::HW_IF_POSITION, &actuators_[i].joint.command.position));

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                actuators_[i].name, hardware_interface::HW_IF_VELOCITY, &actuators_[i].joint.command.velocity));

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                actuators_[i].name, HW_IF_CURRENT, &actuators_[i].joint.command.current));
        }
        return command_interfaces;
    }

    return_type EposHardware::perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> & /*stop_interfaces*/)
    {
        for (size_t i = 0; i < actuators_.size(); i++)
        {
            try
            {
                if (actuators_[i].node_handle == nullptr) {
                    return return_type::ERROR;
                }

                for (auto interface : start_interfaces)
                {
                    if (interface.find(actuators_[i].name) != std::string::npos)
                    {
                        if (interface.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
                        {
                            if (!use_dummy_)
                            {
                                VCS_WRAPPER_NA(VCS_ActivateProfilePositionMode, actuators_[i].node_handle);
                            }

                            actuators_[i].command_interface_name = hardware_interface::HW_IF_POSITION;
                        }
                        else if (interface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
                        {
                            if (!use_dummy_)
                            {
                                VCS_WRAPPER_NA(VCS_ActivateProfileVelocityMode, actuators_[i].node_handle);
                            }

                            actuators_[i].command_interface_name = hardware_interface::HW_IF_VELOCITY;
                        }
                        else if (interface.find(epos_hardware::HW_IF_CURRENT) != std::string::npos)
                        {
                            if (!use_dummy_)
                            {
                                VCS_WRAPPER_NA(VCS_ActivateCurrentMode, actuators_[i].node_handle);
                            }

                            actuators_[i].command_interface_name = epos_hardware::HW_IF_CURRENT;
                        }
                    }
                }
            }
            catch (const EposException &error)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), error.what());
                return return_type::ERROR;
            }

            RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Initialize command interface of motor controller '" << actuators_[i].name << "' to '" << actuators_[i].command_interface_name << "'");
        }

        return return_type::OK;
    }

    CallbackReturn EposHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "start");
        for (size_t i = 0; i < actuators_.size(); i++)
        {
            if (use_dummy_ && std::isnan(actuators_[i].joint.state.position))
            {
                actuators_[i].joint.state.position = 0.0;
                actuators_[i].joint.state.velocity = 0.0;
                actuators_[i].joint.state.current = 0.0;
            }
        }

        if (!use_dummy_)
        {
            for (size_t i = 0; i < actuators_.size(); i++)
            {
                try
                {
                    VCS_WRAPPER_NA(VCS_SetEnableState, actuators_[i].node_handle);
                }
                catch (const EposException &error)
                {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Cannot enable motor controller for joint '" << actuators_[i].name << "': " << error.what());
                    return CallbackReturn::ERROR;
                }
            }
        }

        read(rclcpp::Time{}, rclcpp::Duration(0, 0));
        reset_interfaces();
        write(rclcpp::Time{}, rclcpp::Duration(0, 0));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EposHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "stop");
        // TODO: move stop code here once https://github.com/ros-controls/ros2_control/issues/472 is solved
        if (stop() != return_type::OK)
        {
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    return_type EposHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        if (use_dummy_)
        {
            return return_type::OK;
        }

        try
        {
            for (size_t i = 0; i < actuators_.size(); i++)
            {
                int position_raw, velocity_raw;
                short current_raw;

                // Only publish position when encoder resolution is set
                if (actuators_[i].encoder_resolution != 0)
                {
                    VCS_WRAPPER(VCS_GetPositionIs, actuators_[i].node_handle, &position_raw);
                    actuators_[i].actuator.state.position = (static_cast<double>(position_raw) * M_PI_2 / actuators_[i].encoder_resolution); // Quat-counts -> rad
                }

                VCS_WRAPPER(VCS_GetVelocityIs, actuators_[i].node_handle, &velocity_raw);
                actuators_[i].actuator.state.velocity = (static_cast<double>(velocity_raw) * M_PI / 30.); // rpm -> rad/s

                VCS_WRAPPER(VCS_GetCurrentIs, actuators_[i].node_handle, &current_raw);
                auto s = (actuators_[i].reversed) ? -1.0 : 1.0;
                actuators_[i].joint.state.current = current_raw * s / 1000.; // mA -> A

                actuators_[i].actuator.transmission_passthrough_.position = actuators_[i].actuator.state.position;
                actuators_[i].actuator.transmission_passthrough_.velocity = actuators_[i].actuator.state.velocity;

                actuators_[i].transmission->actuator_to_joint();

                actuators_[i].joint.state.position = actuators_[i].joint.transmission_passthrough_.position;
                actuators_[i].joint.state.velocity = actuators_[i].joint.transmission_passthrough_.velocity;
            }
        }
        catch (const EposException &error)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), error.what());
            return return_type::ERROR;
        }

        return return_type::OK;
    }

    return_type EposHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        if (use_dummy_)
        {
            return return_type::OK;
        }

        try
        {
            for (size_t i = 0; i < actuators_.size(); i++)
            {
                actuators_[i].joint.transmission_passthrough_.position = actuators_[i].joint.command.position;
                actuators_[i].joint.transmission_passthrough_.velocity = actuators_[i].joint.command.velocity;

                actuators_[i].transmission->joint_to_actuator();

                actuators_[i].actuator.command.position = actuators_[i].actuator.transmission_passthrough_.position;
                actuators_[i].actuator.command.velocity = actuators_[i].actuator.transmission_passthrough_.velocity;

                if (actuators_[i].command_interface_name == hardware_interface::HW_IF_POSITION)
                {
                    if (std::isnan(actuators_[i].actuator.command.position))
                    {
                        continue;
                    }

                    // rad -> quad-counts of the encoder
                    long cmd = static_cast<long>(actuators_[i].actuator.command.position * 2 * actuators_[i].encoder_resolution / M_PI);

                    VCS_WRAPPER(VCS_MoveToPosition, actuators_[i].node_handle, cmd, true, true);
                }
                else if (actuators_[i].command_interface_name == hardware_interface::HW_IF_VELOCITY)
                {
                    if (std::isnan(actuators_[i].actuator.command.current))
                    {
                        continue;
                    }

                    long cmd = static_cast<long>(actuators_[i].actuator.command.velocity * 30. / M_PI);

                    if (cmd == 0L)
                    {
                        VCS_WRAPPER_NA(VCS_HaltVelocityMovement, actuators_[i].node_handle);
                    }
                    else
                    {
                        VCS_WRAPPER(VCS_MoveWithVelocity, actuators_[i].node_handle, cmd);
                    }
                }
                else if (actuators_[i].command_interface_name == epos_hardware::HW_IF_CURRENT)
                {
                    if (std::isnan(actuators_[i].actuator.command.current))
                    {
                        continue;
                    }

                    long cmd = actuators_[i].joint.command.current * 1000; // # A -> mA
                    VCS_WRAPPER(VCS_SetCurrentMustEx, actuators_[i].node_handle, cmd);
                }
            }
        }
        catch (const EposException &error)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), error.what());
            return return_type::ERROR;
        }

        return return_type::OK;
    }

    return_type EposHardware::stop()
    {
        if (!use_dummy_)
        {
            for (size_t i = 0; i < actuators_.size(); i++)
            {
                try
                {
                    // Halt movement when needed
                    if (actuators_[i].command_interface_name == hardware_interface::HW_IF_POSITION)
                    {
                        VCS_WRAPPER_NA(VCS_HaltPositionMovement, actuators_[i].node_handle);
                    }
                    else if (actuators_[i].command_interface_name == hardware_interface::HW_IF_VELOCITY)
                    {
                        VCS_WRAPPER_NA(VCS_HaltVelocityMovement, actuators_[i].node_handle);
                    }

                    VCS_WRAPPER_NA(VCS_SetDisableState, actuators_[i].node_handle);
                }
                catch (const EposException &error)
                {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(HW_NAME), "Cannot disable motor controller for joint '" << actuators_[i].name << "': " << error.what());
                    return return_type::ERROR;
                }
            }
        }

        return return_type::OK;
    }

    return_type EposHardware::reset_interfaces()
    {
        auto reset_interface = [](InterfaceData &interface_data, double value)
        {
            interface_data.velocity = value;
            interface_data.position = value;
            interface_data.current = value;
        };

        for (size_t i = 0; i < actuators_.size(); i++)
        {
            reset_interface(actuators_[i].joint.command, 0.0);
            reset_interface(actuators_[i].joint.state, 0.0);
            reset_interface(actuators_[i].joint.transmission_passthrough_, std::numeric_limits<double>::quiet_NaN());
            reset_interface(actuators_[i].actuator.command, 0.0);
            reset_interface(actuators_[i].actuator.state, 0.0);
            reset_interface(actuators_[i].actuator.transmission_passthrough_, std::numeric_limits<double>::quiet_NaN());
        }

        return return_type::OK;
    }

    return_type EposHardware::clearDeviceErrors(NodeHandlePtr node_handle)
    {
        unsigned char num_errors;

        VCS_WRAPPER(VCS_GetNbOfDeviceError, node_handle, &num_errors);

        if (num_errors > 0)
        {
            for (size_t i = 1; i <= num_errors; ++i)
            {
                unsigned int error_number;
                VCS_WRAPPER(VCS_GetDeviceErrorCode, node_handle, i, &error_number);
                RCLCPP_INFO_STREAM(rclcpp::get_logger(HW_NAME), "Clearing EPOS Device Error: 0x" << std::hex << error_number);
            }

            VCS_WRAPPER_NA(VCS_ClearFault, node_handle);
            VCS_WRAPPER(VCS_GetNbOfDeviceError, node_handle, &num_errors);

            if (num_errors > 0)
            {
                RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Not all faults were cleared");
                return return_type::ERROR;
            }
        }

        return return_type::OK;
    }

} // namespace epos_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    epos_hardware::EposHardware, hardware_interface::SystemInterface)
