#ifndef BONXAI__VISIBILITY_CONTROL_H_
#define BONXAI__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BONXAI_EXPORT __attribute__ ((dllexport))
    #define BONXAI_IMPORT __attribute__ ((dllimport))
  #else
    #define BONXAI_EXPORT __declspec(dllexport)
    #define BONXAI_IMPORT __declspec(dllimport)
  #endif
  #ifdef BONXAI_BUILDING_LIBRARY
    #define BONXAI_PUBLIC BONXAI_EXPORT
  #else
    #define BONXAI_PUBLIC BONXAI_IMPORT
  #endif
  #define BONXAI_PUBLIC_TYPE BONXAI_PUBLIC
  #define BONXAI_LOCAL
#else
  #define BONXAI_EXPORT __attribute__ ((visibility("default")))
  #define BONXAI_IMPORT
  #if __GNUC__ >= 4
    #define BONXAI_PUBLIC __attribute__ ((visibility("default")))
    #define BONXAI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BONXAI_PUBLIC
    #define BONXAI_LOCAL
  #endif
  #define BONXAI_PUBLIC_TYPE
#endif

#endif  // BONXAI__VISIBILITY_CONTROL_H_
