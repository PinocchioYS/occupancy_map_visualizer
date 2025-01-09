CMAKE_MINIMUM_REQUIRED( VERSION 3.5 )
PROJECT( occupancy_map_visualizer )

##### Compile options #####
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##### Find dependencies #####
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(octomap REQUIRED)
find_package(superray-octomap REQUIRED)
find_package(superray-quadmap REQUIRED)
find_package(superray-gridmap3d REQUIRED)
find_package(superray-gridmap2d REQUIRED)

###########
## Build ##
###########
include_directories(
#    include
    ${OCTOMAP_INCLUDE_DIRS}
    ${SUPERRAY_OCTOMAP_INCLUDE_DIRS}
    ${SUPERRAY_QUADMAP_INCLUDE_DIRS}
    ${SUPERRAY_GRIDMAP3D_INCLUDE_DIRS}
    ${SUPERRAY_GRIDMAP2D_INCLUDE_DIRS}
)

##### OctoMap Visualizer Node #####
add_executable(octomap_visualizer_node
	src/octomap_visualizer_node.cpp
)
ament_target_dependencies(octomap_visualizer_node
  rclcpp
  visualization_msgs
)
target_link_libraries(octomap_visualizer_node
  ${OCTOMAP_LIBRARIES}
	${SUPERRAY_OCTOMAP_LIBRARIES}
)

##### QuadMap Visualizer Node #####
add_executable(quadmap_visualizer_node
  src/quadmap_visualizer_node.cpp
)
ament_target_dependencies(quadmap_visualizer_node
  rclcpp
  visualization_msgs
)
target_link_libraries(quadmap_visualizer_node
  ${SUPERRAY_QUADMAP_LIBRARIES}
)

##### GridMap3D Visualizer Node #####
add_executable(gridmap3d_visualizer_node
  src/gridmap3d_visualizer_node.cpp
)
ament_target_dependencies(gridmap3d_visualizer_node
  rclcpp
  visualization_msgs
)
target_link_libraries(gridmap3d_visualizer_node
  ${SUPERRAY_GRIDMAP3D_LIBRARIES}
)

##### GridMap2D Visualizer Node #####
add_executable(gridmap2d_visualizer_node
  src/gridmap2d_visualizer_node.cpp
)
ament_target_dependencies(gridmap2d_visualizer_node
  rclcpp
  nav_msgs
  visualization_msgs
)
target_link_libraries(gridmap2d_visualizer_node
  ${SUPERRAY_GRIDMAP2D_LIBRARIES}
)

##### Install commands #####
install(TARGETS
  octomap_visualizer_node
  quadmap_visualizer_node
  gridmap3d_visualizer_node
  gridmap2d_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  samples
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()