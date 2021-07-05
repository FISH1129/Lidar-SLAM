find_package(OpenCV REQUIRED)
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBRARIES})