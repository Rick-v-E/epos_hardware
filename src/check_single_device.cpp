/*
Copyright (c) 2023 Rick-v-E. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#include <iostream>

#include "epos_hardware/utils.hpp"
#include "epos_hardware/Definitions.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

int main(int argc, char **argv)
{
    uint64_t serial_number;

    if (argc != 2)
    {
        std::cerr << "Not enough options." << std::endl;
        std::cerr << "Usage: ros2 run epos_hardware test_single_device serial_number" << std::endl;
        return 1;
    }

    std::cerr << "Searching for motor " << argv[1] << "..." << std::endl;

    SerialNumberFromHex(argv[1], &serial_number);

    EposFactory epos_factory;
    NodeHandlePtr node_handle_;
    unsigned int error_code = 0;
    unsigned char num_errors;

    node_handle_ = epos_factory.CreateNodeHandle("EPOS2", "MAXON SERIAL V2", "USB", serial_number, &error_code);

    if (!node_handle_)
    {
        std::cout << "Could not find motor" << std::endl;
    }

    std::cout << "Found motor " << argv[1] << std::endl;

    try
    {
        VCS_WRAPPER_NA(VCS_SetDisableState, node_handle_);

        std::cout << "Querying faults..." << std::endl;

        VCS_WRAPPER(VCS_GetNbOfDeviceError, node_handle_, &num_errors);

        for (size_t i = 1; i <= num_errors; ++i)
        {
            unsigned int error_number;
            VCS_WRAPPER(VCS_GetDeviceErrorCode, node_handle_, i, &error_number);

            std::cout << "EPOS Device Error: 0x" << std::hex << error_number << std::endl;
        }

        if (num_errors > 0)
        {
            std::cout << "Clearing faults..." << std::endl;
            VCS_WRAPPER_NA(VCS_ClearFault, node_handle_);

            std::cout << "Cleared faults" << std::endl;
        }
        else
        {
            std::cout << "No faults found" << std::endl;
        }

        VCS_WRAPPER(VCS_GetNbOfDeviceError, node_handle_, &num_errors);

        if (num_errors > 0)
        {
            std::cerr << "Not all faults were cleared" << std::endl;
            return 0;
        }

        unsigned int acceleration, deceleration;
        const double conversionFactor = std::pow(2 * M_PI / 60.0, 2);
        VCS_WRAPPER(VCS_GetVelocityProfile, node_handle_, &acceleration, &deceleration);

        double acceleration_rad = acceleration * conversionFactor;
        double deceleration_rad = deceleration * conversionFactor;

        std::cout << "Profile velocity: acceleration->" << acceleration_rad << "rad/s2 deceleration->" << deceleration_rad << "rad/s2" << std::endl;


        std::cout << "Enabling Motor" << std::endl;
        VCS_WRAPPER_NA(VCS_SetEnableState, node_handle_);
        VCS_WRAPPER_NA(VCS_ActivateProfileVelocityMode, node_handle_);

        std::cout << "Start moving Motor for 10 seconds" << std::endl;
        VCS_WRAPPER(VCS_MoveWithVelocity, node_handle_, 1000.0);

        int velocity_raw;
        for (size_t i = 0; i < 10; i++)
        {
            VCS_WRAPPER(VCS_GetVelocityIs, node_handle_, &velocity_raw);
            std::cout << "Moving with velocity of " << velocity_raw << std::endl;
            sleep(1);
        }

        VCS_WRAPPER_NA(VCS_HaltVelocityMovement, node_handle_);
        VCS_WRAPPER_NA(VCS_SetDisableState, node_handle_);
    }
    catch (const EposException &error)
    {
        std::cerr << error.what();
        return 1;
    }

    return 0;
}
