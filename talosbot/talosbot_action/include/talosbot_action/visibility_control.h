#ifndef TALOSBOT_ACTION__VISIBILITY_CONTROL_H_
#define TALOSBOT_ACTION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TALOSBOT_ACTION_EXPORT __attribute__((dllexport))
#define TALOSBOT_ACTION_IMPORT __attribute__((dllimport))
#else
#define TALOSBOT_ACTION_EXPORT __declspec(dllexport)
#define TALOSBOT_ACTION_IMPORT __declspec(dllimport)
#endif
#ifdef TALOSBOT_ACTION_BUILDING_DLL
#define TALOSBOT_ACTION_PUBLIC TALOSBOT_ACTION_EXPORT
#else
#define TALOSBOT_ACTION_PUBLIC TALOSBOT_ACTION_IMPORT
#endif
#define TALOSBOT_ACTION_PUBLIC_TYPE TALOSBOT_ACTION_PUBLIC
#define TALOSBOT_ACTION_LOCAL
#else
#define TALOSBOT_ACTION_EXPORT __attribute__((visibility("default")))
#define TALOSBOT_ACTION_IMPORT
#if __GNUC__ >= 4
#define TALOSBOT_ACTION_PUBLIC __attribute__((visibility("default")))
#define TALOSBOT_ACTION_LOCAL __attribute__((visibility("hidden")))
#else
#define TALOSBOT_ACTION_PUBLIC
#define TALOSBOT_ACTION_LOCAL
#endif
#define TALOSBOT_ACTION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TALOSBOT_ACTION__VISIBILITY_CONTROL_H_