#include <ros/ros.h>

#include <gridmap2D_superray/SuperRayGrid2D.h>
#include <gridmap2D_cullingregion/CullingRegionGrid2D.h>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

// NOTE: Marker uses this small value to draw a square since it requires a volumetric visualization.
const double THICKNESS_SCALE = 0.001;
const double VIS_HEIGHT      = THICKNESS_SCALE / 2.0;

/*
 *  This function generates a visualization message using the occupancy probabilities of all cells of GridMap.
 *  RViz visualizes this message as a set of the gray-scale squares, black(occupied, 1.0) -- white(free, 0.0).
 */
void get_occupancy_grid_msg(gridmap2D::Grid2D* _gridmap2d, nav_msgs::OccupancyGrid& _occupancy_grid_msg, const std::string& _fixed_frame_id = "map");

/*
 *  This function generates a visualization message using the nodes classified to occupied state.
 *  RViz shows the occupied nodes as a set of the blue-colored pixels.
 */
void get_occupied_nodes_msg(gridmap2D::Grid2D* _gridmap2d, visualization_msgs::Marker& _occupied_cells_msg, const std::string& _fixed_frame_id = "map");

/*
 *  This function generates a visualization message using the nodes classified to free state.
 *  RViz visualizes the free nodes to a set of the green pixels.
 */
void get_free_nodes_msg(gridmap2D::Grid2D* _gridmap2d, visualization_msgs::Marker& _free_cells_msg, const std::string& _fixed_frame_id = "map");


int main(int argc, char** argv)
{
    // Initialize the visualization node
    ros::init(argc, argv, "gridmap2d_visualizer_node");
    ros::NodeHandle nh;
    ros::Publisher occupancy_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/superray/gridmap2d/occupancy_grid", 1);
    ros::Publisher occupied_cells_publisher = nh.advertise<visualization_msgs::Marker>("/superray/gridmap2d/occupied_cells", 1);
    ros::Publisher free_cells_publisher     = nh.advertise<visualization_msgs::Marker>("/superray/gridmap2d/free_cells", 1);

    // Load parameters
    std::string fixed_frame_id, gridmap2d_filename;
    nh.getParam("fixed_frame_id", fixed_frame_id);
    nh.getParam("gridmap2d_filename", gridmap2d_filename);

    if(fixed_frame_id.empty()) {
        ROS_ERROR("No fixed frame id");
        return -1;
    }

    if(gridmap2d_filename.empty()) {
        ROS_ERROR("No gridmap2d filename");
        return -1;
    }

    std::string file_extension = gridmap2d_filename.substr(gridmap2d_filename.length() - 4, gridmap2d_filename.length());
    if(file_extension != ".bg2" && file_extension != ".og2") {
        ROS_ERROR("Incorrect file extension");
        ROS_INFO("File extensions of gridmap2D file should be \".bg2\" or \".og2\"");
        return -1;
    }

    // 1. Load the occupancy grid
    gridmap2D::Grid2D* grid2d = new gridmap2D::Grid2D(0.1); // Resolution 0.1 is dummy value for initialization.
    if(file_extension == ".bg2")
        grid2d->readBinary(gridmap2d_filename);
    else if(file_extension == ".og2") {
        gridmap2D::AbstractGrid2D* grid = gridmap2D::AbstractGrid2D::read(gridmap2d_filename);
        if(grid->getGridType() == "SuperRayGrid2D")
            grid2d = (gridmap2D::Grid2D*)dynamic_cast<gridmap2D::SuperRayGrid2D*>(grid);
        else if(grid->getGridType() == "CullingRegionGrid2D")
            grid2d = (gridmap2D::Grid2D*)dynamic_cast<gridmap2D::CullingRegionGrid2D*>(grid);
    }

    if(grid2d->size() <= 0){
        ROS_WARN("No grid cell");
        return -1;
    }

    // Generate some messages for visualization
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    get_occupancy_grid_msg(grid2d, occupancy_grid_msg, fixed_frame_id);

    visualization_msgs::Marker occupied_cells_msg;
    get_occupied_nodes_msg(grid2d, occupied_cells_msg, fixed_frame_id);

    visualization_msgs::Marker free_cells_msg;
    get_free_nodes_msg(grid2d, free_cells_msg, fixed_frame_id);

    // Publish the messages
    ROS_INFO("GridMap2D: %s", gridmap2d_filename.c_str());
    ROS_INFO("Cells: %d", (int)grid2d->size());
    while(nh.ok()){
        if(occupancy_grid_publisher.getNumSubscribers() > 0)
            occupancy_grid_publisher.publish(occupancy_grid_msg);

        if(occupied_cells_publisher.getNumSubscribers() > 0)
            occupied_cells_publisher.publish(occupied_cells_msg);

        if(free_cells_publisher.getNumSubscribers() > 0)
            free_cells_publisher.publish(free_cells_msg);

        ros::spinOnce();
    }

    delete grid2d;

    return 0;
}

void get_occupancy_grid_msg(gridmap2D::Grid2D* _gridmap2d, nav_msgs::OccupancyGrid& _occupancy_grid_msg, const std::string& _fixed_frame_id)
{
    // Compute a region of quadtree visualization
    double min_x, min_y, max_x, max_y;
    _gridmap2d->getMetricMin(min_x, min_y);
    _gridmap2d->getMetricMax(max_x, max_y);
    gridmap2D::Grid2DKey min_key = _gridmap2d->coordToKey(min_x, min_y);
    gridmap2D::Grid2DKey max_key = _gridmap2d->coordToKey(max_x, max_y);

    // Initialize a visualization message (nav_msgs::OccupancyGrid)
    _occupancy_grid_msg.header.frame_id = _fixed_frame_id;
    _occupancy_grid_msg.header.stamp = ros::Time::now();

    _occupancy_grid_msg.info.resolution = (float)_gridmap2d->getResolution();

    _occupancy_grid_msg.info.origin.position.x = _gridmap2d->keyToCoord(min_key[0]) - (_gridmap2d->getResolution() / 2.0);
    _occupancy_grid_msg.info.origin.position.y = _gridmap2d->keyToCoord(min_key[1]) - (_gridmap2d->getResolution() / 2.0);
    _occupancy_grid_msg.info.origin.position.z = 0.0;
    _occupancy_grid_msg.info.origin.orientation.x = 0.0;
    _occupancy_grid_msg.info.origin.orientation.y = 0.0;
    _occupancy_grid_msg.info.origin.orientation.z = 0.0;
    _occupancy_grid_msg.info.origin.orientation.w = 1.0;

    _occupancy_grid_msg.info.width = (unsigned int)(max_key[0] - min_key[0] + 1);
    _occupancy_grid_msg.info.height = (unsigned int)(max_key[1] - min_key[1] + 1);
    _occupancy_grid_msg.data.resize(_occupancy_grid_msg.info.width * _occupancy_grid_msg.info.height, -1);

    // Insert occupancy data to the message
    for(auto it = _gridmap2d->getGrid()->begin(); it != _gridmap2d->getGrid()->end(); it++){
        gridmap2D::Grid2DKey node_key = it->first;
        int cell_x = node_key[0] - min_key[0];
        int cell_y = node_key[1] - min_key[1];
        int cell_idx = cell_y * _occupancy_grid_msg.info.width + cell_x;

        _occupancy_grid_msg.data[cell_idx] = (char)(it->second->getOccupancy() * 100.0);
    }
}

void get_occupied_nodes_msg(gridmap2D::Grid2D* _gridmap2d, visualization_msgs::Marker& _occupied_cells_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::Marker)
    double size = _gridmap2d->getResolution();

    _occupied_cells_msg.header.frame_id = _fixed_frame_id;
    _occupied_cells_msg.header.stamp = ros::Time::now();
    _occupied_cells_msg.type = visualization_msgs::Marker::CUBE_LIST;
    _occupied_cells_msg.scale.x = size;
    _occupied_cells_msg.scale.y = size;
    _occupied_cells_msg.scale.z = THICKNESS_SCALE;

    _occupied_cells_msg.color.r = 0.0;
    _occupied_cells_msg.color.g = 0.0;
    _occupied_cells_msg.color.b = 1.0;
    _occupied_cells_msg.color.a = 1.0;

    _occupied_cells_msg.action = visualization_msgs::Marker::ADD;

    // Insert data of occupied nodes to the message
    for(auto it = _gridmap2d->getGrid()->begin(); it != _gridmap2d->getGrid()->end(); it++) {
        if(_gridmap2d->isNodeOccupied(it->second)) {
            geometry_msgs::Point center;
            center.x = _gridmap2d->keyToCoord(it->first[0]);
            center.y = _gridmap2d->keyToCoord(it->first[1]);
            center.z = VIS_HEIGHT;

            _occupied_cells_msg.points.push_back(center);
        }
    }
}

void get_free_nodes_msg(gridmap2D::Grid2D* _gridmap2d, visualization_msgs::Marker& _free_cells_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::Marker)
    double size = _gridmap2d->getResolution();

    _free_cells_msg.header.frame_id = _fixed_frame_id;
    _free_cells_msg.header.stamp = ros::Time::now();
    _free_cells_msg.type = visualization_msgs::Marker::CUBE_LIST;
    _free_cells_msg.scale.x = size;
    _free_cells_msg.scale.y = size;
    _free_cells_msg.scale.z = THICKNESS_SCALE;

    _free_cells_msg.color.r = 0.0;
    _free_cells_msg.color.g = 1.0;
    _free_cells_msg.color.b = 0.0;
    _free_cells_msg.color.a = 0.7;

    _free_cells_msg.action = visualization_msgs::Marker::ADD;

    // Insert data of occupied nodes to the message
    for(auto it = _gridmap2d->getGrid()->begin(); it != _gridmap2d->getGrid()->end(); it++) {
        if(_gridmap2d->isNodeAtThreshold(it->second) && !_gridmap2d->isNodeOccupied(it->second)) {
            geometry_msgs::Point center;
            center.x = _gridmap2d->keyToCoord(it->first[0]);
            center.y = _gridmap2d->keyToCoord(it->first[1]);
            center.z = VIS_HEIGHT;

            _free_cells_msg.points.push_back(center);
        }
    }
}