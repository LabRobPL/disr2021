#-----------------------------------------------------------------------------
#
# VTKConfig.cmake - VTK CMake configuration file for external projects.
#
# This file is configured by VTK and used by the UseVTK.cmake module
# to load VTK's settings for an external project.

# Compute the installation prefix from this VTKConfig.cmake file location.
set(_vtk_installed_prefix "/opt/drake/lib/cmake/vtk-8.1")
set(_vtk_requested_prefix "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(_vtk_installed_prefix_full "${_vtk_installed_prefix}" REALPATH)
get_filename_component(_vtk_requested_prefix_full "${_vtk_requested_prefix}" REALPATH)
if (_vtk_installed_prefix_full STREQUAL _vtk_requested_prefix_full)
  set(VTK_INSTALL_PREFIX "/opt/drake")
else ()
  set(VTK_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}")
  get_filename_component(VTK_INSTALL_PREFIX "${VTK_INSTALL_PREFIX}" PATH)
  get_filename_component(VTK_INSTALL_PREFIX "${VTK_INSTALL_PREFIX}" PATH)
  get_filename_component(VTK_INSTALL_PREFIX "${VTK_INSTALL_PREFIX}" PATH)
endif ()
set(VTK_MODULES_DIR "${VTK_INSTALL_PREFIX}/lib/cmake/vtk-8.1/Modules")

set (__vtk_install_tree TRUE)

if (CMAKE_VERSION VERSION_LESS "3.3")
    message(FATAL_ERROR "VTK now requires CMake 3.3 or newer")
  endif()

#-----------------------------------------------------------------------------
# Minimum compiler version check: GCC >= 4.6
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.6)
  message(FATAL_ERROR "GCC 4.6 or later is required.")
endif ()

#-----------------------------------------------------------------------------
# Minimum compiler version check: LLVM Clang >= 3.0
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.0)
  message(FATAL_ERROR "LLVM Clang 3.0 or later is required.")
endif ()

#-----------------------------------------------------------------------------
# Minimum compiler version check: Apple Clang >= 3.0 (Xcode 4.2)
if (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.0)
  message(FATAL_ERROR "Apple Clang 3.0 or later is required.")
endif ()

#-----------------------------------------------------------------------------
# Minimum compiler version check: Microsoft C/C++ >= 17.0 (aka VS 2012 aka VS 11.0)
if ("x${CMAKE_CXX_COMPILER_ID}" STREQUAL "xMSVC" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_LESS 17.0)
  message(FATAL_ERROR "Microsoft Visual Studio 2012 or later is required.")
endif ()

#-----------------------------------------------------------------------------
# Minimum compiler version check: Intel C++ (ICC) >= 14
if (CMAKE_CXX_COMPILER_ID STREQUAL "Intel" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14.0)
  message(FATAL_ERROR "Intel C++ (ICC) 14.0 or later is required.")
endif ()

# The C and C++ flags added by VTK to the cmake-configured flags.
SET(VTK_REQUIRED_C_FLAGS "")
SET(VTK_REQUIRED_CXX_FLAGS "")
SET(VTK_REQUIRED_EXE_LINKER_FLAGS "")
SET(VTK_REQUIRED_SHARED_LINKER_FLAGS "")
SET(VTK_REQUIRED_MODULE_LINKER_FLAGS "")

# The VTK version number
SET(VTK_MAJOR_VERSION "8")
SET(VTK_MINOR_VERSION "1")
SET(VTK_BUILD_VERSION "1")

# The location of the UseVTK.cmake file.
SET(VTK_CMAKE_DIR "${VTK_INSTALL_PREFIX}/lib/cmake/vtk-8.1")
SET(VTK_USE_FILE "${VTK_CMAKE_DIR}/UseVTK.cmake")

# The rendering backend VTK was configured to use.
set(VTK_RENDERING_BACKEND "OpenGL2")

if (__vtk_install_tree)
  if (WIN32)
    set (VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/bin")
  else ()
    set (VTK_RUNTIME_DIRS "${VTK_INSTALL_PREFIX}/lib")
  endif ()
else()
  if (WIN32)
    set (VTK_RUNTIME_DIRS "/vtk-build/bin")
  else ()
    set (VTK_RUNTIME_DIRS "/vtk-build/lib")
  endif ()
endif()

# Setup VTK-m if it was enabled
set(VTK_HAS_VTKM false)
if(VTK_HAS_VTKM AND __vtk_install_tree)
  set(VTKM_CMAKE_DIR "${VTK_CMAKE_DIR}")
  get_filename_component(VTKM_CMAKE_DIR "${VTKM_CMAKE_DIR}" PATH)
  find_package(VTKm
               PATHS "${CMAKE_CURRENT_LIST_DIR}"
                     "${VTKM_CMAKE_DIR}"
                     "${VTK_RUNTIME_DIRS}"
               NO_DEFAULT_PATH
               )
endif()


#-----------------------------------------------------------------------------
# Load requested modules.

# List of available VTK modules.
set(VTK_MODULES_ENABLED "vtkkwiml;vtksys;vtkCommonCore;vtkCommonMath;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkCommonDataModel;vtkCommonColor;vtkCommonExecutionModel;vtkCommonComputationalGeometry;vtkFiltersCore;vtkFiltersGeneral;vtkImagingCore;vtkImagingFourier;vtkalglib;vtkFiltersStatistics;vtkFiltersExtraction;vtkInfovisCore;vtkFiltersGeometry;vtkFiltersSources;vtkRenderingCore;vtkzlib;vtkfreetype;vtkRenderingFreeType;vtkRenderingContext2D;vtkChartsCore;vtkDICOMParser;vtklz4;vtkIOCore;vtkIOLegacy;vtkexpat;vtkIOXMLParser;vtkDomainsChemistry;vtkUtilitiesEncodeString;vtkglew;vtkRenderingOpenGL2;vtkDomainsChemistryOpenGL2;vtkIOXML;vtkUtilitiesHashSource;vtkParallelCore;vtkFiltersAMR;vtkFiltersFlowPaths;vtkFiltersGeneric;vtkImagingSources;vtkFiltersHybrid;vtkFiltersHyperTree;vtkImagingGeneral;vtkFiltersImaging;vtkFiltersModeling;vtkFiltersParallel;vtkFiltersParallelImaging;vtkFiltersPoints;vtkFiltersProgrammable;vtkPython;vtkWrappingTools;vtkWrappingPythonCore;vtkFiltersPython;vtkFiltersSMP;vtkFiltersSelection;vtkFiltersTexture;vtkFiltersTopology;vtkverdict;vtkFiltersVerdict;vtkInteractionStyle;vtkGUISupportQt;vtksqlite;vtkIOSQL;vtkGUISupportQtSQL;vtkMetaIO;vtkjpeg;vtkpng;vtktiff;vtkIOImage;vtkImagingHybrid;vtkInfovisLayout;vtkImagingColor;vtkRenderingAnnotation;vtkRenderingVolume;vtkInteractionWidgets;vtkViewsCore;vtklibproj4;vtkGeovisCore;vtkhdf5;vtkIOAMR;vtkIOEnSight;vtknetcdf;vtkexodusII;vtkIOExodus;vtkgl2ps;vtkRenderingGL2PSOpenGL2;vtklibharu;vtkIOExport;vtkIOExportOpenGL2;vtkIOGeometry;vtkIOImport;vtklibxml2;vtkIOInfovis;vtkIOLSDyna;vtkIOMINC;vtkoggtheora;vtkIOMovie;vtknetcdfcpp;vtkIONetCDF;vtkIOPLY;vtkjsoncpp;vtkIOParallel;vtkIOParallelXML;vtkIOTecplotTable;vtkIOVideo;vtkImagingMath;vtkImagingMorphological;vtkImagingStatistics;vtkImagingStencil;vtkInteractionImage;vtkRenderingContextOpenGL2;vtkRenderingImage;vtkRenderingLOD;vtkRenderingLabel;vtkRenderingSceneGraph;vtkRenderingVolumeOpenGL2;vtkRenderingVolumeAMR;vtkRenderingOSPRay;vtkRenderingQt;vtkViewsContext2D;vtkViewsInfovis;vtkViewsQt")

# Import VTK targets.
set(VTK_CONFIG_TARGETS_FILE "${VTK_INSTALL_PREFIX}/lib/cmake/vtk-8.1/VTKTargets.cmake")
if(NOT TARGET vtkCommonCore)
  include("${VTK_CONFIG_TARGETS_FILE}")
endif()

# Load module interface macros.
include("${VTK_INSTALL_PREFIX}/lib/cmake/vtk-8.1/vtkModuleAPI.cmake")

# Compute set of requested modules.
if(VTK_FIND_COMPONENTS)
  # Specific modules requested by find_package(VTK).
  set(VTK_MODULES_REQUESTED "${VTK_FIND_COMPONENTS}")
else()
  # No specific modules requested.  Use all of them.
  set(VTK_MODULES_REQUESTED "${VTK_MODULES_ENABLED}")
endif()

# Load requested modules and their dependencies into variables:
#  VTK_DEFINITIONS     = Preprocessor definitions
#  VTK_LIBRARIES       = Libraries to link
#  VTK_INCLUDE_DIRS    = Header file search path
#  VTK_LIBRARY_DIRS    = Library search path (for outside dependencies)
vtk_module_config(VTK ${VTK_MODULES_REQUESTED})

#-----------------------------------------------------------------------------

# VTK global configuration options.
SET(VTK_BUILD_SHARED_LIBS "ON")
SET(VTK_LEGACY_REMOVE "ON")
SET(VTK_LEGACY_SILENT "OFF")
SET(VTK_WRAP_PYTHON "ON")
SET(VTK_WRAP_TCL "OFF")
SET(VTK_WRAP_JAVA "OFF")
SET(VTK_QT_VERSION "5")
SET(VTK_ENABLE_KITS "OFF")

# Do not add options or information here that is specific to a
# particular module.  Instead set <module>_EXPORT_OPTIONS and/or
# <module>_EXPORT_CODE_BUILD and <module>_EXPORT_CODE_INSTALL
# at the top of the module CMakeLists.txt file.
