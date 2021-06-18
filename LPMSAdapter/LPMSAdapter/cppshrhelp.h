#ifndef CPPSHRHELP
#define CPPSHRHELP

#ifdef _MSC_VER
# define DLL_EXPORT __declspec(dllexport)
#elif __GNUC__ >= 4
# define DLL_EXPORT __attribute__ ((visibility("default")))
#else
# define DLL_EXPORT
#endif

#endif