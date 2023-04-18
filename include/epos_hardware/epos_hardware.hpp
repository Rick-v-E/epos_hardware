/*
Copyright (c) 2023 Rick-v-E. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#ifndef EPOS_HARDWARE__EPOS_HARDWARE_HPP_
#define EPOS_HARDWARE__EPOS_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <bitset>
#include <chrono>
#include <map>
#include <tuple>
#include <functional>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "epos_hardware/utils.hpp"
#include "epos_hardware/visibility_control.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace epos_hardware
{

  constexpr char HW_IF_CURRENT[] = "current"; // Unit: A

  struct InterfaceData
  {
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double current = std::numeric_limits<double>::quiet_NaN();
  };

  struct Interface
  {
    InterfaceData state{};
    InterfaceData command{};
    InterfaceData transmission_passthrough_{};
  };

  struct Actuator
  {
    std::string name;
    std::string command_interface_name;
    std::string device_name;
    std::string protocol_stack_name;
    std::string interface_name;
    uint64_t serial_number;
    double profile_acceleration;
    double profile_deceleration;
    int encoder_resolution;
    bool clear_faults = false;
    bool reversed = false;

    NodeHandlePtr node_handle;
    std::shared_ptr<transmission_interface::Transmission> transmission;

    Interface joint;
    Interface actuator;
  };

  class EposHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EposHardware)

    EPOS_HARDWARE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    EPOS_HARDWARE_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    EPOS_HARDWARE_PUBLIC
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    EPOS_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    EPOS_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    EPOS_HARDWARE_PUBLIC
    return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

    EPOS_HARDWARE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    EPOS_HARDWARE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    EPOS_HARDWARE_PUBLIC
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    EPOS_HARDWARE_PUBLIC
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    return_type stop();
    return_type reset_interfaces();
    return_type clearDeviceErrors(NodeHandlePtr node_handle);

    std::vector<Actuator> actuators_;
    EposFactory epos_factory_;
    bool use_dummy_ = false;
  };

} // namespace motionmind_hardware

#endif