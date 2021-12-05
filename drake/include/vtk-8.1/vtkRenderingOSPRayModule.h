
#ifndef VTKRENDERINGOSPRAY_EXPORT_H
#define VTKRENDERINGOSPRAY_EXPORT_H

#ifdef VTKRENDERINGOSPRAY_STATIC_DEFINE
#  define VTKRENDERINGOSPRAY_EXPORT
#  define VTKRENDERINGOSPRAY_NO_EXPORT
#else
#  ifndef VTKRENDERINGOSPRAY_EXPORT
#    ifdef vtkRenderingOSPRay_EXPORTS
        /* We are building this library */
#      define VTKRENDERINGOSPRAY_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKRENDERINGOSPRAY_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKRENDERINGOSPRAY_NO_EXPORT
#    define VTKRENDERINGOSPRAY_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKRENDERINGOSPRAY_DEPRECATED
#  define VTKRENDERINGOSPRAY_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKRENDERINGOSPRAY_DEPRECATED_EXPORT VTKRENDERINGOSPRAY_EXPORT __attribute__ ((__deprecated__))
#  define VTKRENDERINGOSPRAY_DEPRECATED_NO_EXPORT VTKRENDERINGOSPRAY_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKRENDERINGOSPRAY_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkRenderingCoreModule.h"
#include "vtkRenderingOpenGL2Module.h"
#include "vtkRenderingVolumeModule.h"

/* AutoInit implementations.  */
#if defined(vtkRenderingOSPRay_INCLUDE)
# include vtkRenderingOSPRay_INCLUDE
#endif
#if defined(vtkRenderingOSPRay_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkRenderingOSPRay)
#endif

#endif
