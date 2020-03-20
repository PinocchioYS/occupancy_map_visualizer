ROS visualization tool for occupancy maps of SuperRay library
=============================================================

This ROS package provides the examples of visualizing various types of occupancy maps - octree, quadtree, grid3D, grid2D - 
that [SuperRay library](https://github.com/PinocchioYS/SuperRay) supports.
The example nodes publish various ROS messages according to the visualization data (see the below table), 
and we can use a RViz tool to visualize these messages.

| Occupancy map | Occupancy     | Occupied nodes | Frees nodes    |
| ------------- | ------------- | -------------- | -------------- |
| OctoMap       | MarkerArray   | MarkerArray    | MarkerArray    |
| QuadMap       | MarkerArray   | MarkerArray    | MarkerArray    |
| Grid3D        | Marker        | Marker         | Marker         |
| Grid2D        | OccupancyGrid | Marker         | Marker         |

BUILD
-----
This package requires to install [SuperRay library](https://github.com/PinocchioYS/SuperRay).
For example of installing the library, run:

    cd ~/Downloads && git clone https://github.com/PinocchioYS/SuperRay.git
    mkdir SuperRay/build && cd SuperRay/build
    cmake ..
    sudo make install

You can build the visualization through the following commands:

    cd ~/catkin_ws/src
    git clone https://github.com/PinocchioYS/occupancy_map_visualizer.git
    cd ~/catkin_ws && catkin_make
  
RUN EXAMPLES
------------
You can run some examples as follows, for example, if you want to visualize a quadmap:

    roslaunch occupancy_map_visualizer quadmap_visualization.launch
  
It will visualize the occupancy, occupied and free nodes of the quadmap in RViz.
As an option, you can see a map saved in your custom directory like:

    roslaunch occupancy_map_visualizer quadmap_visualization.launch filename:=YOUR_DIR/YOUR_FILE.ot2
  
If you have any problem or issue, notice it at [here](https://github.com/PinocchioYS/occupancy_map_visualizer/issues).
  

