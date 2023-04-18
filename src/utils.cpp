/*
Copyright (c) 2023 Rick-v-E. All rights reserved.
Copyright (c) 2018 yoshito-n-students
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#include "epos_hardware/utils.hpp"

#define MAX_STRING_SIZE 1000

using hardware_interface::return_type;

EposException::EposException(const std::string &what_arg)
    : std::runtime_error(what_arg), has_error_code_(false), error_code_(0) {}

EposException::EposException(const std::string &what_arg, const unsigned int error_code)
    : std::runtime_error(what_arg + " (" + toErrorInfo(error_code) + ")"), has_error_code_(true),
      error_code_(error_code) {}

EposException::~EposException() throw() {}

bool EposException::hasErrorCode() const { return has_error_code_; }

unsigned int EposException::getErrorCode() const { return error_code_; }

std::string EposException::toErrorInfo(const unsigned int error_code)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << error_code;
  char error_info[1024];
  if (VCS_GetErrorInfo(error_code, error_info, 1024))
  {
    oss << ": " << error_info;
  }
  return oss.str();
}

bool SerialNumberFromHex(const std::string &str, uint64_t *serial_number)
{
  std::stringstream ss;
  ss << std::hex << str;
  ss >> *serial_number;
  return true;
}

int GetErrorInfo(unsigned int error_code, std::string *error_string)
{
  char buffer[MAX_STRING_SIZE];
  int result;

  result = VCS_GetErrorInfo(error_code, buffer, MAX_STRING_SIZE);
  if (!result)
  {
    *error_string = buffer;
  }
  return result;
}

int GetDeviceNameList(std::vector<std::string> *device_names, unsigned int *error_code)
{
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; // BOOL
  int result;

  result = VCS_GetDeviceNameSelection(true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if (!result)
    return result;
  device_names->push_back(buffer);

  while (!end_of_selection)
  {
    result = VCS_GetDeviceNameSelection(false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result)
      return result;
    device_names->push_back(buffer);
  }

  return 1;
}

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string> *protocol_stack_names, unsigned int *error_code)
{
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; // BOOL
  int result;

  result = VCS_GetProtocolStackNameSelection((char *)device_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if (!result)
    return result;
  protocol_stack_names->push_back(buffer);

  while (!end_of_selection)
  {
    result = VCS_GetProtocolStackNameSelection((char *)device_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result)
      return result;
    protocol_stack_names->push_back(buffer);
  }

  return 1;
}

int GetInterfaceNameList(const std::string device_name, const std::string protocol_stack_name, std::vector<std::string> *interface_names, unsigned int *error_code)
{
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; // BOOL
  int result;

  result = VCS_GetInterfaceNameSelection((char *)device_name.c_str(), (char *)protocol_stack_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if (!result)
    return result;
  interface_names->push_back(buffer);

  while (!end_of_selection)
  {
    result = VCS_GetInterfaceNameSelection((char *)device_name.c_str(), (char *)protocol_stack_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result)
      return result;
    interface_names->push_back(buffer);
  }

  return 1;
}

int GetPortNameList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name, std::vector<std::string> *port_names, unsigned int *error_code)
{
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; // BOOL
  int result;

  result = VCS_GetPortNameSelection((char *)device_name.c_str(), (char *)protocol_stack_name.c_str(), (char *)interface_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if (!result)
    return result;
  port_names->push_back(buffer);

  while (!end_of_selection)
  {
    result = VCS_GetPortNameSelection((char *)device_name.c_str(), (char *)protocol_stack_name.c_str(), (char *)interface_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if (!result)
      return result;
    port_names->push_back(buffer);
  }

  return 1;
}

int GetBaudrateList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                    const std::string port_name, std::vector<unsigned int> *baudrates, unsigned int *error_code)
{
  unsigned int baudrate;
  int end_of_selection; // BOOL
  int result;

  result = VCS_GetBaudrateSelection((char *)device_name.c_str(), (char *)protocol_stack_name.c_str(), (char *)interface_name.c_str(), (char *)port_name.c_str(), true, &baudrate, &end_of_selection, error_code);
  if (!result)
    return result;
  baudrates->push_back(baudrate);

  while (!end_of_selection)
  {
    result = VCS_GetBaudrateSelection((char *)device_name.c_str(), (char *)protocol_stack_name.c_str(), (char *)interface_name.c_str(), (char *)port_name.c_str(), false, &baudrate, &end_of_selection, error_code);
    if (!result)
      return result;
    baudrates->push_back(baudrate);
  }

  return 1;
}

EposFactory::EposFactory()
{
}

DeviceHandlePtr EposFactory::CreateDeviceHandle(const std::string device_name,
                                                const std::string protocol_stack_name,
                                                const std::string interface_name,
                                                const std::string port_name,
                                                unsigned int *error_code)
{
  const std::string key = device_name + '/' + protocol_stack_name + '/' + interface_name + '/' + port_name;
  DeviceHandlePtr handle;
  if (!(handle = existing_handles[key].lock()))
  { // Handle exists
    void *raw_handle = VCS_OpenDevice((char *)device_name.c_str(), (char *)protocol_stack_name.c_str(), (char *)interface_name.c_str(), (char *)port_name.c_str(), error_code);
    if (!raw_handle) // failed to open device
      return DeviceHandlePtr();

    handle = DeviceHandlePtr(new DeviceHandle(raw_handle));
    existing_handles[key] = handle;
  }

  return handle;
}

NodeHandlePtr EposFactory::CreateNodeHandle(const std::string device_name,
                                            const std::string protocol_stack_name,
                                            const std::string interface_name,
                                            const uint64_t serial_number,
                                            unsigned int *error_code)
{
  std::vector<EnumeratedNode> nodes;
  EnumerateNodes(device_name, protocol_stack_name, interface_name, &nodes, error_code);

  return CreateNodeHandle(nodes, serial_number, error_code);
}

NodeHandlePtr EposFactory::CreateNodeHandle(std::vector<EnumeratedNode> nodes,
                                            const uint64_t serial_number,
                                            unsigned int *error_code)
{
  for (const EnumeratedNode &node : nodes)
  {
    if (node.serial_number == serial_number)
    {
      return CreateNodeHandle(node, error_code);
    }
  }
  return NodeHandlePtr();
}

NodeHandlePtr EposFactory::CreateNodeHandle(const EnumeratedNode &node,
                                            unsigned int *error_code)
{
  DeviceHandlePtr device_handle = CreateDeviceHandle(node.device_name, node.protocol_stack_name, node.interface_name, node.port_name, error_code);
  if (!device_handle)
    return NodeHandlePtr();
  return NodeHandlePtr(new NodeHandle(device_handle, node.node_id));
}

int EposFactory::EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                                const std::string port_name, std::vector<EnumeratedNode> *nodes, unsigned int *error_code)
{
  DeviceHandlePtr handle;
  if (!(handle = CreateDeviceHandle(device_name, protocol_stack_name, interface_name, port_name, error_code)))
  {
    return 0;
  }
  for (unsigned short i = 1; i < 127; ++i)
  {
    EnumeratedNode node;
    node.device_name = device_name;
    node.protocol_stack_name = protocol_stack_name;
    node.interface_name = interface_name;
    node.port_name = port_name;
    node.node_id = i;
    if (!VCS_GetVersion(handle->ptr, i, &node.hardware_version, &node.software_version, &node.application_number, &node.application_version, error_code))
    {
      return 1;
    }
    unsigned int bytes_read;
    if (!VCS_GetObject(handle->ptr, i, 0x2004, 0x00, &node.serial_number, 8, &bytes_read, error_code))
    {
      node.serial_number = 0;
    }
    nodes->push_back(node);
  }
  return 1;
}

int EposFactory::EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                                std::vector<EnumeratedNode> *nodes, unsigned int *error_code)
{
  std::vector<std::string> port_names;
  if (GetPortNameList(device_name, protocol_stack_name, interface_name, &port_names, error_code))
  {
    for (const std::string &port_name : port_names)
    {
      if (!EnumerateNodes(device_name, protocol_stack_name, interface_name, port_name, nodes, error_code))
      {
        return 0;
      }
    }
    return 1;
  }
  else
    return 0;
}

return_type getParamString(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, std::string *variable_ptr)
{
  if (parameter_map.find(parameter_name) == parameter_map.end())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EposHardware"), "Cannot find parameter '" << parameter_name << "'!");
    return return_type::ERROR;
  }

  *variable_ptr = parameter_map.at(parameter_name);
  return return_type::OK;
}

return_type getParamInt(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, int *variable_ptr)
{
  if (parameter_map.find(parameter_name) == parameter_map.end())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EposHardware"), "Cannot find parameter '" << parameter_name << "'!");
    return return_type::ERROR;
  }

  int value = std::stoi(parameter_map.at(parameter_name));
  *variable_ptr = value;
  return return_type::OK;
}

return_type getParamDouble(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, double *variable_ptr)
{
  if (parameter_map.find(parameter_name) == parameter_map.end())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("EposHardware"), "Cannot find parameter '" << parameter_name << "'!");
    return return_type::ERROR;
  }

  int value = std::stod(parameter_map.at(parameter_name));
  *variable_ptr = value;
  return return_type::OK;
}

return_type getParamBoolean(std::unordered_map<std::string, std::string> parameter_map, std::string parameter_name, bool *variable_ptr)
{
  bool value = (parameter_map.find(parameter_name) != parameter_map.end() && parameter_map.at(parameter_name) == "true");
  *variable_ptr = value;

  return return_type::OK;
}
