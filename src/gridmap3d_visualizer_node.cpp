#include <ros/ros.h>

#include <gridmap3D_superray/SuperRayGrid3D.h>
#include <gridmap3D_cullingregion/CullingRegionGrid3D.h>

#include <visualization_msgs/Marker.h>

/*
 *  This function generates a visualization message using the occupancy probabilities of all cells of GridMap.
 *  RViz visualizes this message as a set of the gray-scale cubes, black(occupied, 1.0) -- white(free, 0.0).
 */
void get_occupancy_grid_msg(gridmap3D::Grid3D* _gridmap3d, visualization_msgs::Marker& _occupancy_grid_msg, const std::string& _fixed_frame_id = "map");

/*
 *  This function generates a visualization message using the nodes classified to occupied state.
 *  RViz shows the occupied nodes as a set of the blue-colored voxels.
 */
void get_occupied_nodes_msg(gridmap3D::Grid3D* _gridmap3d, visualization_msgs::Marker& _occupied_cells_msg, const std::string& _fixed_frame_id = "map");

/*
 *  This function generates a visualization message using the nodes classified to free state.
 *  RViz visualizes the free nodes to a set of the green voxels.
 */
void get_free_nodes_msg(gridmap3D::Grid3D* _gridmap3d, visualization_msgs::Marker& _free_cells_msg, const std::string& _fixed_frame_id = "map");

// Function that returns a color according to given different values
inline std_msgs::ColorRGBA get_occupancy_color(const double _occupancy);


int main(int argc, char** argv)
{
    // Initialize the visualization node
    ros::init(argc, argv, "gridmap3d_visualizer_node");
    ros::NodeHandle nh;
    ros::Publisher occupancy_grid_publisher = nh.advertise<visualization_msgs::Marker>("/superray/gridmap3d/occupancy_grid", 1);
    ros::Publisher occupied_cells_publisher = nh.advertise<visualization_msgs::Marker>("/superray/gridmap3d/occupied_cells", 1);
    ros::Publisher free_cells_publisher     = nh.advertise<visualization_msgs::Marker>("/superray/gridmap3d/free_cells", 1);

    // Load parameters
    std::string fixed_frame_id, gridmap3d_filename;
    nh.getParam("fixed_frame_id", fixed_frame_id);
    nh.getParam("gridmap3d_filename", gridmap3d_filename);

    if(fixed_frame_id.empty()) {
        ROS_ERROR("No fixed frame id");
        return -1;
    }

    if(gridmap3d_filename.empty()) {
        ROS_ERROR("No gridmap3d filename");
        return -1;
    }

    std::string file_extension = gridmap3d_filename.substr(gridmap3d_filename.length() - 4, gridmap3d_filename.length());
    if(file_extension != ".bg3" && file_extension != ".og3") {
        ROS_ERROR("Incorrect file extension");
        ROS_INFO("File extensions of gridmap3D file should be \".bg3\" or \".og3\"");
        return -1;
    }

    // 1. Load the occupancy grid
    gridmap3D::Grid3D* grid3d = new gridmap3D::Grid3D(0.1); // Resolution 0.1 is dummy value for initialization.
    if(file_extension == ".bg3")
        grid3d->readBinary(gridmap3d_filename);
    else if(file_extension == ".og3") {
        gridmap3D::AbstractGrid3D* grid = gridmap3D::AbstractGrid3D::read(gridmap3d_filename);
        if(grid->getGridType() == "SuperRayGrid3D")
            grid3d = (gridmap3D::Grid3D*)dynamic_cast<gridmap3D::SuperRayGrid3D*>(grid);
        else if(grid->getGridType() == "CullingRegionGrid3D")
            grid3d = (gridmap3D::Grid3D*)dynamic_cast<gridmap3D::CullingRegionGrid3D*>(grid);
    }

    if(grid3d->size() <= 0){
        ROS_WARN("No grid cell");
        return -1;
    }

    // Generate some messages for visualization
    visualization_msgs::Marker occupancy_grid_msg;
    get_occupancy_grid_msg(grid3d, occupancy_grid_msg, fixed_frame_id);

    visualization_msgs::Marker occupied_cells_msg;
    get_occupied_nodes_msg(grid3d, occupied_cells_msg, fixed_frame_id);

    visualization_msgs::Marker free_cells_msg;
    get_free_nodes_msg(grid3d, free_cells_msg, fixed_frame_id);

    // Publish the messages
    ROS_INFO("GridMap3D: %s", gridmap3d_filename.c_str());
    ROS_INFO("Cells: %d", (int)grid3d->size());
    while(nh.ok()){
        if(occupancy_grid_publisher.getNumSubscribers() > 0)
            occupancy_grid_publisher.publish(occupancy_grid_msg);

        if(occupied_cells_publisher.getNumSubscribers() > 0)
            occupied_cells_publisher.publish(occupied_cells_msg);

        if(free_cells_publisher.getNumSubscribers() > 0)
            free_cells_publisher.publish(free_cells_msg);

        ros::spinOnce();
    }

    delete grid3d;

    return 0;
}

void get_occupancy_grid_msg(gridmap3D::Grid3D* _gridmap3d, visualization_msgs::Marker& _occupancy_grid_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::Marker)
    double size = _gridmap3d->getResolution();

    _occupancy_grid_msg.header.frame_id = _fixed_frame_id;
    _occupancy_grid_msg.header.stamp = ros::Time::now();
    _occupancy_grid_msg.type = visualization_msgs::Marker::CUBE_LIST;
    _occupancy_grid_msg.scale.x = size;
    _occupancy_grid_msg.scale.y = size;
    _occupancy_grid_msg.scale.z = size;

    _occupancy_grid_msg.action = visualization_msgs::Marker::ADD;

    // Insert occupancy data to the message
    for(auto it = _gridmap3d->getGrid()->begin(); it != _gridmap3d->getGrid()->end(); it++) {
        geometry_msgs::Point center;
        center.x = _gridmap3d->keyToCoord(it->first[0]);
        center.y = _gridmap3d->keyToCoord(it->first[1]);
        center.z = _gridmap3d->keyToCoord(it->first[2]);

        std_msgs::ColorRGBA color = get_occupancy_color(it->second->getOccupancy());

        _occupancy_grid_msg.points.push_back(center);
        _occupancy_grid_msg.colors.push_back(color);
    }
}

void get_occupied_nodes_msg(gridmap3D::Grid3D* _gridmap3d, visualization_msgs::Marker& _occupied_cells_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::Marker)
    double size = _gridmap3d->getResolution();

    _occupied_cells_msg.header.frame_id = _fixed_frame_id;
    _occupied_cells_msg.header.stamp = ros::Time::now();
    _occupied_cells_msg.type = visualization_msgs::Marker::CUBE_LIST;
    _occupied_cells_msg.scale.x = size;
    _occupied_cells_msg.scale.y = size;
    _occupied_cells_msg.scale.z = size;

    _occupied_cells_msg.color.r = 0.0;
    _occupied_cells_msg.color.g = 0.0;
    _occupied_cells_msg.color.b = 1.0;
    _occupied_cells_msg.color.a = 1.0;

    _occupied_cells_msg.action = visualization_msgs::Marker::ADD;

    // Insert data of occupied nodes to the message
    for(auto it = _gridmap3d->getGrid()->begin(); it != _gridmap3d->getGrid()->end(); it++) {
        if(_gridmap3d->isNodeOccupied(it->second)) {
            geometry_msgs::Point center;
            center.x = _gridmap3d->keyToCoord(it->first[0]);
            center.y = _gridmap3d->keyToCoord(it->first[1]);
            center.z = _gridmap3d->keyToCoord(it->first[2]);

            _occupied_cells_msg.points.push_back(center);
        }
    }
}

void get_free_nodes_msg(gridmap3D::Grid3D* _gridmap3d, visualization_msgs::Marker& _free_cells_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::Marker)
    double size = _gridmap3d->getResolution();

    _free_cells_msg.header.frame_id = _fixed_frame_id;
    _free_cells_msg.header.stamp = ros::Time::now();
    _free_cells_msg.type = visualization_msgs::Marker::CUBE_LIST;
    _free_cells_msg.scale.x = size;
    _free_cells_msg.scale.y = size;
    _free_cells_msg.scale.z = size;

    _free_cells_msg.color.r = 0.0;
    _free_cells_msg.color.g = 1.0;
    _free_cells_msg.color.b = 0.0;
    _free_cells_msg.color.a = 0.7;

    _free_cells_msg.action = visualization_msgs::Marker::ADD;

    // Insert data of occupied nodes to the message
    for(auto it = _gridmap3d->getGrid()->begin(); it != _gridmap3d->getGrid()->end(); it++) {
        if(_gridmap3d->isNodeAtThreshold(it->second) && !_gridmap3d->isNodeOccupied(it->second)) {
            geometry_msgs::Point center;
            center.x = _gridmap3d->keyToCoord(it->first[0]);
            center.y = _gridmap3d->keyToCoord(it->first[1]);
            center.z = _gridmap3d->keyToCoord(it->first[2]);

            _free_cells_msg.points.push_back(center);
        }
    }
}

std_msgs::ColorRGBA get_occupancy_color(const double _occupancy)
{
    double occupancy_color = 1.0 - std::min(std::max(_occupancy, 0.0), 1.0);

    // Color map: gray-scale
    std_msgs::ColorRGBA color;
    color.r = (float)occupancy_color;
    color.g = (float)occupancy_color;
    color.b = (float)occupancy_color;
    color.a = 1.0;

    return color;
}