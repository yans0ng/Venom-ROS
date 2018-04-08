cmake_minimum_required(VERSION 2.8.3)
project(venom_offb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	mavros_msgs
	eigen_conversions
	venom_perception
)

## NOTE: For package exporting, not needed right now
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs mavros_msgs eigen_conversions eigen_conversions venom_perception
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${venom_perception_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/Navigator.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Declare a C++ executable
add_executable(hover src/hover.cpp)
target_link_libraries(hover ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(vision_pose_converter src/vision_pose_converter.cpp)
target_link_libraries(vision_pose_converter ${catkin_LIBRARIES})

add_executable(circle src/circle.cpp)
target_link_libraries(circle ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(hunt_cloud src/hunt_cloud.cpp)
target_link_libraries(hunt_cloud ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(hunt_servo src/hunt_servo.cpp)
target_link_libraries(hunt_servo ${catkin_LIBRAREIS} ${PROJECT_NAME})
