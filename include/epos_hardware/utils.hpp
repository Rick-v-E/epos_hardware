/*
Copyright (c) 2023 Rick-v-E. All rights reserved.
Copyright (c) 2018 yoshito-n-students
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#ifndef EPOS_HARDWARE_UTILS_HPP_
#define EPOS_HARDWARE_UTILS_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include <stdexcept>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "epos_hardware/Definitions.h"

class EposException : public std::runtime_error
{
public:
  EposException(const std::string &what_arg);
  EposException(const std::string &what_arg, const unsigned int error_code);
  virtual ~EposException() throw();

  bool hasErrorCode() const;
  unsigned int getErrorCode() const;

  static std::string toErrorInfo(const unsigned int error_code);

private:
  bool has_error_code_;
  unsigned int error_code_;
};

bool SerialNumberFromHex(const std::string &str, uint64_t *serial_number);

int GetErrorInfo(unsigned int error_code, std::string *error_string);

int GetDeviceNameList(std::vector<std::string> *device_names, unsigned int *error_code);

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string> *protocol_stack_names, unsigned int *error_code);

int GetInterfaceNameList(const std::string device_name, const std::string protocol_stack_name,
                         std::vector<std::string> *interface_names, unsigned int *error_code);

int GetPortNameList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                    std::vector<std::string> *port_names, unsigned int *error_code);

int GetBaudrateList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                    const std::string port_name, std::vector<unsigned int> *port_names, unsigned int *error_code);

hardware_interface::return_type getParamString(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, std::string *variable_ptr);
hardware_interface::return_type getParamInt(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, int *variable_ptr);
hardware_interface::return_type getParamBoolean(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, bool *variable_ptr);

// An object that wraps a handle and closes it when destroyed
class DeviceHandle
{
public:
  DeviceHandle(void *ptr) : ptr(ptr) {}
  ~DeviceHandle()
  {
    unsigned int error_code;
    VCS_CloseDevice(const_cast<void *>(ptr), &error_code);
  }
  void *const ptr;
};
typedef std::shared_ptr<DeviceHandle> DeviceHandlePtr;

class NodeHandle
{
public:
  NodeHandle(DeviceHandlePtr device_handle, unsigned short node_id)
      : device_handle(device_handle), node_id(node_id) {}
  const DeviceHandlePtr device_handle;
  const unsigned short node_id;
};
typedef std::shared_ptr<NodeHandle> NodeHandlePtr;

typedef struct
{
  std::string device_name;
  std::string protocol_stack_name;
  std::string interface_name;
  std::string port_name;
  unsigned short node_id;
  uint64_t serial_number;
  unsigned short hardware_version;
  unsigned short software_version;
  unsigned short application_number;
  unsigned short application_version;
} EnumeratedNode;

class EposFactory
{
public:
  EposFactory();
  DeviceHandlePtr CreateDeviceHandle(const std::string device_name,
                                     const std::string protocol_stack_name,
                                     const std::string interface_name,
                                     const std::string port_name,
                                     unsigned int *error_code);

  NodeHandlePtr CreateNodeHandle(std::vector<EnumeratedNode> nodes,
                                 const uint64_t serial_number,
                                 unsigned int *error_code);

  NodeHandlePtr CreateNodeHandle(const std::string device_name,
                                 const std::string protocol_stack_name,
                                 const std::string interface_name,
                                 uint64_t serial_number,
                                 unsigned int *error_code);

  NodeHandlePtr CreateNodeHandle(const EnumeratedNode &node,
                                 unsigned int *error_code);

  int EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                     const std::string port_name, std::vector<EnumeratedNode> *devices, unsigned int *error_code);

  int EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                     std::vector<EnumeratedNode> *devices, unsigned int *error_code);

private:
  std::map<std::string, std::weak_ptr<DeviceHandle>> existing_handles;
};

#define VCS_FALSE 0

// call a VCS_xxx function or die
#define VCS(func, ...)                                \
  do                                                  \
  {                                                   \
    unsigned int _error_code;                         \
    if (func(__VA_ARGS__, &_error_code) == VCS_FALSE) \
    {                                                 \
      throw ::EposException(#func, _error_code);      \
    }                                                 \
  } while (false)

#define VCS_WRAPPER_NA(func, node_handle, ...) VCS(func, node_handle->device_handle->ptr, node_handle->node_id)
#define VCS_WRAPPER(func, node_handle, ...) VCS(func, node_handle->device_handle->ptr, node_handle->node_id, __VA_ARGS__)

#endif
