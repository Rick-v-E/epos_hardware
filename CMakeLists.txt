cmake_minimum_required(VERSION 3.5)
project(epos_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Processor classification is based on install.sh of the epos command library
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86_64/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86_64/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i386"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i486"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i586"
       OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i686")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/intel/x86/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv6")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v6/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v6/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v7/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v7/libEposCmd.so.6.8.1.0)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v8/libftd2xx.so.1.4.8)
  set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/arm/v8/libEposCmd.so.6.8.1.0)
else()
  message(FATAL_ERROR "Unsupported processor type: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)

## COMPILE
add_library(EposUtils SHARED src/utils.cpp)
target_include_directories(EposUtils PRIVATE include)
target_link_libraries(EposUtils ${ftd2xx_LIBRARY} ${EposCmd_LIBRARY})
ament_target_dependencies(EposUtils
  hardware_interface
  rclcpp
)

add_library(${PROJECT_NAME} SHARED src/epos_hardware.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} EposUtils)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  transmission_interface
  rclcpp_lifecycle
  lifecycle_msgs
  pluginlib
  rclcpp
)

add_executable(check_single_device src/check_single_device.cpp)
target_include_directories(check_single_device PRIVATE include)
target_link_libraries(check_single_device EposUtils)

add_executable(list_available_devices src/list_available_devices.cpp)
target_include_directories(list_available_devices PRIVATE include)
target_link_libraries(list_available_devices EposUtils)

pluginlib_export_plugin_description_file(hardware_interface epos_hardware.xml)

# INSTALL
install(FILES ${ftd2xx_LIBRARY} DESTINATION lib/ RENAME libftd2xx.so)
install(FILES ${EposCmd_LIBRARY} DESTINATION lib/ RENAME libEposCmd.so)
install(TARGETS ${PROJECT_NAME} EposUtils DESTINATION lib)
install(TARGETS check_single_device list_available_devices DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY misc/ DESTINATION lib/)
install(PROGRAMS bin/install_udev_rules DESTINATION lib/${PROJECT_NAME})

## EXPORTS
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} EposUtils)
ament_export_dependencies(hardware_interface transmission_interface pluginlib rclcpp)

ament_package()
