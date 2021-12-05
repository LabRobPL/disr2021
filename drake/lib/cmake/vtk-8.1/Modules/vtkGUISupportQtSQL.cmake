set(vtkGUISupportQtSQL_LOADED 1)
set(vtkGUISupportQtSQL_DEPENDS "vtkCommonCore;vtkIOSQL;vtksys")
set(vtkGUISupportQtSQL_LIBRARIES "vtkGUISupportQtSQL")
set(vtkGUISupportQtSQL_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-8.1")
set(vtkGUISupportQtSQL_LIBRARY_DIRS "")
set(vtkGUISupportQtSQL_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/lib")
set(vtkGUISupportQtSQL_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkGUISupportQtSQLHierarchy.txt")
set(vtkGUISupportQtSQL_KIT "")
set(vtkGUISupportQtSQL_TARGETS_FILE "")
set(vtkGUISupportQtSQL_EXCLUDE_FROM_WRAPPING 1)

if(NOT Qt5_DIR)
  set(Qt5_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5")
endif()
find_package(Qt5 REQUIRED QUIET COMPONENTS Sql Widgets)


