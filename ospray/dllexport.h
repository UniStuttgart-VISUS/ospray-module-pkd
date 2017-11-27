#ifndef DLL_EXPORT_H_
#define DLL_EXPORT_H_

#if defined (_WIN32)
#if defined(ospray_module_pkd_EXPORTS)
#define OSPRAY_PKD_EXPORT __declspec(dllexport)
#else
#define OSPRAY_PKD_EXPORT __declspec(dllimport)
#endif
#else
#define OSPRAY_PKD_EXPORT
#endif

#endif