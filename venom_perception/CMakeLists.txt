cmake_minimum_required(VERSION 2.8.3)
project(venom_perception)

add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED) # Must be found before other packages
find_package(
  catkin REQUIRED
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs pcl_ros pcl_conversions image_transport cv_bridge
  DEPENDS OpenCV
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

## Implementation of C++ header files
add_library(${PROJECT_NAME} include/${PROJECT_NAME}/Zed.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
)


## Declare a C++ executable
add_executable(depth_map src/depth_map.cpp)
target_link_libraries(depth_map ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(save_zed src/save_zed.cpp)
target_link_libraries(save_zed ${catkin_LIBRARIES})

add_executable(local_map src/local_map.cpp)
target_link_libraries(local_map ${catkin_LIBRARIES})

add_executable(detect_cloud src/detect_cloud.cpp)
target_link_libraries(detect_cloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})
