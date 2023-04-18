
/*
Copyright (c) 2023 Rick-v-E. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#include <iostream>
#include <iomanip>
#include <vector>

#include "epos_hardware/utils.hpp"
#include "epos_hardware/Definitions.h"

void print_error(unsigned int error_code, std::string failed_function_name)
{
    std::string error_string;
    if (GetErrorInfo(error_code, &error_string))
    {
        std::cerr << "Could not " << failed_function_name << ": " << error_string << std::endl;
    }
    else
    {
        std::cerr << "Could not " << failed_function_name << std::endl;
    }
}

int main(int argc, char **argv)
{
    bool skip_rs232 = true;
    if (argc == 2)
    {
        if (!std::string(argv[1]).compare("--rs232"))
        {
            skip_rs232 = false;
        }
        else
        {
            std::cerr << "Unknown option: " << argv[1] << std::endl;
            std::cerr << "Usage: ros2 run epos_hardware list_available_devices [--rs232]" << std::endl;
            return 1;
        }
    }
    else if (argc > 1)
    {
        std::cerr << "Unknown options" << std::endl;
        std::cerr << "Usage: ros2 run epos_hardware list_available_devices [--rs232]" << std::endl;
        return 1;
    }

    EposFactory epos_factory;
    unsigned int error_code = 0;

    std::vector<EnumeratedNode> all_devices;

    std::vector<std::string> device_names;
    if (GetDeviceNameList(&device_names, &error_code))
    {
        for (auto device_name : device_names)
        {
            std::vector<std::string> protocol_stack_names;
            if (GetProtocolStackNameList(device_name, &protocol_stack_names, &error_code))
            {
                for (auto protocol_stack_name : protocol_stack_names)
                {
                    std::vector<std::string> interface_names;
                    if (GetInterfaceNameList(device_name, protocol_stack_name, &interface_names, &error_code))
                    {
                        for (auto interface_name : interface_names)
                        {
                            if (skip_rs232 && interface_name == "RS232")
                            {
                                std::cout << "\t\t\tSkipping RS232" << std::endl;
                                continue;
                            }
                            std::vector<std::string> port_names;
                            if (GetPortNameList(device_name, protocol_stack_name, interface_name, &port_names, &error_code))
                            {
                                for (auto port_name : port_names)
                                {
                                    std::vector<unsigned int> baudrates;
                                    if (GetBaudrateList(device_name, protocol_stack_name, interface_name, port_name, &baudrates, &error_code))
                                    {
                                        DeviceHandlePtr handle = epos_factory.CreateDeviceHandle(device_name, protocol_stack_name, interface_name, port_name, &error_code);

                                        std::vector<EnumeratedNode> devices;
                                        if (epos_factory.EnumerateNodes(device_name, protocol_stack_name, interface_name, port_name, &devices, &error_code))
                                        {
                                            all_devices.insert(std::end(all_devices), std::begin(devices), std::end(devices));
                                        }
                                        else
                                        {
                                            print_error(error_code, "enumerate devices");
                                        }
                                    }
                                    else
                                    {
                                        print_error(error_code, "get baudrates");
                                    }
                                }
                            }
                            else
                            {
                                print_error(error_code, "get port names");
                            }
                        }
                    }
                    else
                    {
                        print_error(error_code, "get interface names");
                    }
                }
            }
            else
            {
                print_error(error_code, "get protocol stack names");
            }
        }
    }
    else
    {
        print_error(error_code, "get device names");
    }

    std::cout
        << std::left << std::setw(14) << "Device name"
        << std::left << std::setw(22) << "Protocol stack name"
        << std::left << std::setw(17) << "Interface name"
        << std::left << std::setw(10) << "Port name"
        << std::left << std::setw(10) << "Node ID"
        << std::left << std::setw(18) << "Serial number"
        << std::left << std::setw(19) << "Hardware version"
        << std::left << std::setw(19) << "Software version"
        << std::left << std::setw(21) << "Application number"
        << std::left << std::setw(22) << "Application version"
        << std::endl;

    for (auto dev : all_devices)
    {
        std::cout
            << std::left << std::setw(14) << dev.device_name
            << std::left << std::setw(22) << dev.protocol_stack_name
            << std::left << std::setw(17) << dev.interface_name
            << std::left << std::setw(10) << dev.port_name
            << std::left << std::setw(10) << std::dec << dev.node_id
            << std::left << std::setw(2) << "0x"
            << std::left << std::setw(16) << std::hex << dev.serial_number
            << std::left << std::setw(2) << "0x"
            << std::left << std::setw(17) << std::hex << dev.hardware_version
            << std::left << std::setw(2) << "0x"
            << std::left << std::setw(17) << std::hex << dev.software_version
            << std::left << std::setw(2) << "0x"
            << std::left << std::setw(19) << std::hex << dev.application_number
            << std::left << std::setw(2) << "0x"
            << std::left << std::setw(19) << std::hex << dev.application_version
            << std::endl;
    }

    if (all_devices.size() == 0)
    {
        std::cout << "No devices found!" << std::endl;
    }

    return 0;
}
