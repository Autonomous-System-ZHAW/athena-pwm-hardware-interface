#ifndef PWM_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define PWM_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PWM_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define PWM_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define PWM_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define PWM_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef PWM_HARDWARE_INTERFACE_BUILDING_DLL
#define PWM_HARDWARE_INTERFACE_PUBLIC PWM_HARDWARE_INTERFACE_EXPORT
#else
#define PWM_HARDWARE_INTERFACE_PUBLIC PWM_HARDWARE_INTERFACE_IMPORT
#endif
#define PWM_HARDWARE_INTERFACE_PUBLIC_TYPE PWM_HARDWARE_INTERFACE_PUBLIC
#define PWM_HARDWARE_INTERFACE_LOCAL
#else
#define PWM_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define PWM_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define PWM_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define PWM_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define PWM_HARDWARE_INTERFACE_PUBLIC
#define PWM_HARDWARE_INTERFACE_LOCAL
#endif
#define PWM_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // PWM_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_