project(vtkSlicer${MODULE_NAME}ModuleVTKWidgets)

set(KIT ${PROJECT_NAME})

set(${KIT}_EXPORT_DIRECTIVE "VTK_SLICER_${MODULE_NAME_UPPER}_MODULE_VTKWIDGETS_EXPORT")

set(${KIT}_INCLUDE_DIRECTORIES
  ${vtkSlicer${MODULE_NAME}ModuleMRML_SOURCE_DIR}
  ${vtkSlicer${MODULE_NAME}ModuleMRML_BINARY_DIR}
  )

set(${KIT}_SRCS
  vtkSlicerLinearTransformWidgetRepresentation.cxx
  vtkSlicerLinearTransformWidgetRepresentation.h
  vtkSlicerLinearTransformWidgetRepresentation3D.cxx
  vtkSlicerLinearTransformWidgetRepresentation3D.h
  vtkSlicerLinearTransformWidgetRepresentation2D.cxx
  vtkSlicerLinearTransformWidgetRepresentation2D.h
  vtkSlicerLinearTransformWidget.cxx
  vtkSlicerLinearTransformWidget.h
  )

# set(${KIT}_TARGET_LIBRARIES
#  vtkSlicer${MODULE_NAME}ModuleMRML
#   )

#-----------------------------------------------------------------------------
SlicerMacroBuildModuleLogic(
  NAME ${KIT}
  EXPORT_DIRECTIVE ${${KIT}_EXPORT_DIRECTIVE}
  INCLUDE_DIRECTORIES ${${KIT}_INCLUDE_DIRECTORIES}
  SRCS ${${KIT}_SRCS}
  TARGET_LIBRARIES ${${KIT}_TARGET_LIBRARIES}
  )
