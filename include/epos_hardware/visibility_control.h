/*
Copyright (c) 2023 Rick-v-E. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/
#ifndef EPOS_HARDWARE__VISIBLITY_CONTROL_H_
#define EPOS_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define EPOS_HARDWARE_EXPORT __attribute__((dllexport))
#define EPOS_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define EPOS_HARDWARE_EXPORT __declspec(dllexport)
#define EPOS_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef EPOS_HARDWARE_BUILDING_DLL
#define EPOS_HARDWARE_PUBLIC EPOS_HARDWARE_EXPORT
#else
#define EPOS_HARDWARE_PUBLIC EPOS_HARDWARE_IMPORT
#endif
#define EPOS_HARDWARE_PUBLIC_TYPE EPOS_HARDWARE_PUBLIC
#define EPOS_HARDWARE_LOCAL
#else
#define EPOS_HARDWARE_EXPORT __attribute__((visibility("default")))
#define EPOS_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define EPOS_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define EPOS_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define EPOS_HARDWARE_PUBLIC
#define EPOS_HARDWARE_LOCAL
#endif
#define EPOS_HARDWARE_PUBLIC_TYPE
#endif

#endif // EPOS_HARDWARE__VISIBLITY_CONTROL_H_