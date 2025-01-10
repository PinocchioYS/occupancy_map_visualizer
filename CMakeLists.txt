CMAKE_MINIMUM_REQUIRED( VERSION 2.8.3 )
PROJECT( occupancy_map_visualizer )

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nav_msgs
	visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(octomap REQUIRED)
find_package(superray-octomap REQUIRED)
find_package(superray-quadmap REQUIRED)
find_package(superray-gridmap3d REQUIRED)
find_package(superray-gridmap2d REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES occupancy_map_visualizer
    CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
#	include
	${catkin_INCLUDE_DIRS}
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
target_link_libraries(octomap_visualizer_node
	${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
	${SUPERRAY_OCTOMAP_LIBRARIES}
)

##### QuadMap Visualizer Node #####
add_executable(quadmap_visualizer_node
  src/quadmap_visualizer_node.cpp
)
target_link_libraries(quadmap_visualizer_node
	${catkin_LIBRARIES}
  ${SUPERRAY_QUADMAP_LIBRARIES}
)

##### GridMap3D Visualizer Node #####
add_executable(gridmap3d_visualizer_node
  src/gridmap3d_visualizer_node.cpp
)
target_link_libraries(gridmap3d_visualizer_node
	${catkin_LIBRARIES}
  ${SUPERRAY_GRIDMAP3D_LIBRARIES}
)

##### GridMap2D Visualizer Node #####
add_executable(gridmap2d_visualizer_node
  src/gridmap2d_visualizer_node.cpp
)
target_link_libraries(gridmap2d_visualizer_node
	${catkin_LIBRARIES}
  ${SUPERRAY_GRIDMAP2D_LIBRARIES}
)