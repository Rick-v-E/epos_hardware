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

        unsigned int profile_velocity_raw, profile_acceleration_raw, profile_deceleration_raw;
        double profile_velocity, profile_acceleration, profile_deceleration;

        VCS_WRAPPER(VCS_GetVelocityProfile, node_handle_, &profile_acceleration_raw, &profile_deceleration_raw);

        profile_acceleration = profile_acceleration_raw * std::pow(2 * M_PI / 60.0, 2);
        profile_deceleration = profile_deceleration_raw * std::pow(2 * M_PI / 60.0, 2);

        std::cout << "Profile velocity: acceleration->" << profile_acceleration << "rad/s2 deceleration->" << profile_deceleration << "rad/s2" << std::endl;

        VCS_WRAPPER(VCS_GetPositionProfile, node_handle_, &profile_velocity_raw, &profile_acceleration_raw, &profile_deceleration_raw);

        profile_velocity = profile_velocity_raw * 2 * M_PI / 60.0;
        profile_acceleration = profile_acceleration_raw * std::pow(2 * M_PI / 60.0, 2);
        profile_deceleration = profile_deceleration_raw * std::pow(2 * M_PI / 60.0, 2);

        std::cout << "Profile position: velocity->" << profile_velocity << "rad/s acceleration->" << profile_acceleration << "rad/s2 deceleration->" << profile_deceleration << "rad/s2" << std::endl;


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
