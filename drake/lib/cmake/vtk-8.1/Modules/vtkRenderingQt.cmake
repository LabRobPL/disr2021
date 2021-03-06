set(vtkRenderingQt_LOADED 1)
set(vtkRenderingQt_DEPENDS "vtkCommonCore;vtkCommonDataModel;vtkCommonExecutionModel;vtkCommonSystem;vtkFiltersSources;vtkFiltersTexture;vtkGUISupportQt;vtkRenderingCore;vtkRenderingLabel")
set(vtkRenderingQt_LIBRARIES "vtkRenderingQt")
set(vtkRenderingQt_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-8.1")
set(vtkRenderingQt_LIBRARY_DIRS "")
set(vtkRenderingQt_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/lib")
set(vtkRenderingQt_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkRenderingQtHierarchy.txt")
set(vtkRenderingQt_KIT "")
set(vtkRenderingQt_TARGETS_FILE "")

if(NOT Qt5_DIR)
  set(Qt5_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5")
endif()
find_package(Qt5 REQUIRED QUIET COMPONENTS Widgets)


