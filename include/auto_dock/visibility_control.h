#ifndef AUTO_DOCK__VISIBILITY_CONTROL_H_
#define AUTO_DOCK__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AUTO_DOCK_EXPORT __attribute__ ((dllexport))
    #define AUTO_DOCK_IMPORT __attribute__ ((dllimport))
  #else
    #define AUTO_DOCK_EXPORT __declspec(dllexport)
    #define AUTO_DOCK_IMPORT __declspec(dllimport)
  #endif
  #ifdef AUTO_DOCK_BUILDING_DLL
    #define AUTO_DOCK_PUBLIC AUTO_DOCK_EXPORT
  #else
    #define AUTO_DOCK_PUBLIC AUTO_DOCK_IMPORT
  #endif
  #define AUTO_DOCK_PUBLIC_TYPE AUTO_DOCK_PUBLIC
  #define AUTO_DOCK_LOCAL
#else
  #define AUTO_DOCK_EXPORT __attribute__ ((visibility("default")))
  #define AUTO_DOCK_IMPORT
  #if __GNUC__ >= 4
    #define AUTO_DOCK_PUBLIC __attribute__ ((visibility("default")))
    #define AUTO_DOCK_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AUTO_DOCK_PUBLIC
    #define AUTO_DOCK_LOCAL
  #endif
  #define AUTO_DOCK_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // AUTO_DOCK__VISIBILITY_CONTROL_H_