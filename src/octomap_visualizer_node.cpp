#include <ros/ros.h>

#include <octomap_superray/SuperRayOcTree.h>
#include <octomap_cullingregion/CullingRegionOcTree.h>

#include <visualization_msgs/MarkerArray.h>

/*
 *  This function generates a visualization message using the occupancy probabilities of all nodes of OctoMap.
 *  RViz visualizes this message as a set of the gray-scale cubes, black(occupied, 1.0) -- white(free, 0.0),
 *  with various sizes due to tree structure of OctoMap.
 */
void get_occupancy_grid_msg(octomap::OcTree* _octree, visualization_msgs::MarkerArray& _occupancy_grid_msg, const std::string& _fixed_frame_id = "map");

/*
 *  This function generates a visualization message using the nodes classified to occupied state.
 *  RViz shows the occupied nodes with various colors depending on depth levels; red(root) -- blue(max_depth).
 */
void get_occupied_nodes_msg(octomap::OcTree* _octree, visualization_msgs::MarkerArray& _occupied_nodes_msg, const std::string& _fixed_frame_id = "map");

/*
 *  This function generates a visualization message using the nodes classified to free state.
 *  RViz visualizes the free nodes to a set of the green cubes with different sizes.
 */
void get_free_nodes_msg(octomap::OcTree* _octree, visualization_msgs::MarkerArray& _free_nodes_msg, const std::string& _fixed_frame_id = "map");

// Function that returns a color according to given different values
inline std_msgs::ColorRGBA get_occupancy_color(const double _occupancy);
inline std_msgs::ColorRGBA get_depth_color(const int _depth, const int _MAX_DEPTH, const int _MIN_DEPTH);


int main(int argc, char** argv)
{
    // Initialize the visualization node
    ros::init(argc, argv, "octomap_visualizer_node");
    ros::NodeHandle nh;
    ros::Publisher occupancy_grid_publisher = nh.advertise<visualization_msgs::MarkerArray>("/superray/octomap/occupancy_grid", 1);
    ros::Publisher occupied_nodes_publisher = nh.advertise<visualization_msgs::MarkerArray>("/superray/octomap/occupied_nodes", 1);
    ros::Publisher free_nodes_publisher     = nh.advertise<visualization_msgs::MarkerArray>("/superray/octomap/free_nodes", 1);

    // Load parameters
    std::string fixed_frame_id, octomap_filename;
    nh.getParam("fixed_frame_id", fixed_frame_id);
    nh.getParam("octomap_filename", octomap_filename);

    if(fixed_frame_id.empty()) {
        ROS_ERROR("No fixed frame id");
        return -1;
    }

    if(octomap_filename.empty()) {
        ROS_ERROR("No octomap filename");
        return -1;
    }

    std::string file_extension = octomap_filename.substr(octomap_filename.length() - 3, octomap_filename.length());
    if(file_extension != ".bt" && file_extension != ".ot") {
        ROS_ERROR("Incorrect file extension");
        ROS_INFO("File extensions of octomap file should be \".bt\" or \".ot\"");
        return -1;
    }

    // Load an octomap
    octomap::OcTree* octree = new octomap::OcTree(0.1); // Resolution 0.1 is dummy value for initialization.
    if(file_extension == ".bt")
        octree->readBinary(octomap_filename);
    else if(file_extension == ".ot") {
        octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(octomap_filename);
        if(tree->getTreeType() == "SuperRayOcTree")
            octree = (octomap::OcTree*)dynamic_cast<octomap::SuperRayOcTree*>(tree);
        else if(tree->getTreeType() == "CullingRegionOcTree")
            octree = (octomap::OcTree*)dynamic_cast<octomap::CullingRegionOcTree*>(tree);
    }

    if(octree->getNumLeafNodes() <= 0) {
        ROS_WARN("No octree node");
        return -1;
    }

    // Generate some messages for visualization
    visualization_msgs::MarkerArray occupancy_grid_msg;
    get_occupancy_grid_msg(octree, occupancy_grid_msg, fixed_frame_id);

    visualization_msgs::MarkerArray occupied_nodes_msg;
    get_occupied_nodes_msg(octree, occupied_nodes_msg, fixed_frame_id);

    visualization_msgs::MarkerArray free_nodes_msg;
    get_free_nodes_msg(octree, free_nodes_msg, fixed_frame_id);

    // Publish the messages
    ROS_INFO("OctoMap: %s", octomap_filename.c_str());
    ROS_INFO("Leaf nodes: %d", (int)octree->getNumLeafNodes());
    while(nh.ok()){
        if(occupancy_grid_publisher.getNumSubscribers() > 0)
            occupancy_grid_publisher.publish(occupancy_grid_msg);

        if(occupied_nodes_publisher.getNumSubscribers() > 0)
            occupied_nodes_publisher.publish(occupied_nodes_msg);

        if(free_nodes_publisher.getNumSubscribers() > 0)
            free_nodes_publisher.publish(free_nodes_msg);

        ros::spinOnce();
    }

    delete octree;

    return 0;
}


void get_occupancy_grid_msg(octomap::OcTree* _octree, visualization_msgs::MarkerArray& _occupancy_grid_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::MarkerArray)
    ros::Time timestamp = ros::Time::now();
    _occupancy_grid_msg.markers.resize(_octree->getTreeDepth()+1);
    for(unsigned int i = 0; i < _occupancy_grid_msg.markers.size(); i++) {
        double size = _octree->getNodeSize(i);

        _occupancy_grid_msg.markers[i].header.frame_id = _fixed_frame_id;
        _occupancy_grid_msg.markers[i].header.stamp = timestamp;
        _occupancy_grid_msg.markers[i].id = i;
        _occupancy_grid_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        _occupancy_grid_msg.markers[i].scale.x = size;
        _occupancy_grid_msg.markers[i].scale.y = size;
        _occupancy_grid_msg.markers[i].scale.z = size;

        _occupancy_grid_msg.markers[i].action = visualization_msgs::Marker::ADD;
    }

    // Insert occupancy data to the message
    for(auto leaf_it = _octree->begin_leafs(); leaf_it != _octree->end_leafs(); leaf_it++) {
        unsigned int marker_idx = leaf_it.getDepth();

        geometry_msgs::Point center;
        center.x = leaf_it.getX();
        center.y = leaf_it.getY();
        center.z = leaf_it.getZ();

        std_msgs::ColorRGBA color = get_occupancy_color(leaf_it->getOccupancy());

        _occupancy_grid_msg.markers[marker_idx].points.push_back(center);
        _occupancy_grid_msg.markers[marker_idx].colors.push_back(color);
    }
}

void get_occupied_nodes_msg(octomap::OcTree* _octree, visualization_msgs::MarkerArray& _occupied_nodes_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::MarkerArray)
    ros::Time timestamp = ros::Time::now();
    _occupied_nodes_msg.markers.resize(_octree->getTreeDepth()+1);
    for(unsigned int i = 0; i < _occupied_nodes_msg.markers.size(); i++) {
        double size = _octree->getNodeSize(i);

        _occupied_nodes_msg.markers[i].header.frame_id = _fixed_frame_id;
        _occupied_nodes_msg.markers[i].header.stamp = timestamp;
        _occupied_nodes_msg.markers[i].id = i;
        _occupied_nodes_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        _occupied_nodes_msg.markers[i].scale.x = size;
        _occupied_nodes_msg.markers[i].scale.y = size;
        _occupied_nodes_msg.markers[i].scale.z = size;

        _occupied_nodes_msg.markers[i].color = get_depth_color(i, (int)_occupied_nodes_msg.markers.size(), 0);

        _occupied_nodes_msg.markers[i].action = visualization_msgs::Marker::ADD;
    }

    // Insert data of occupied nodes to the message
    for(auto leaf_it = _octree->begin_leafs(); leaf_it != _octree->end_leafs(); leaf_it++) {
        if(_octree->isNodeOccupied(*leaf_it)) {
            unsigned int marker_idx = leaf_it.getDepth();

            geometry_msgs::Point center;
            center.x = leaf_it.getX();
            center.y = leaf_it.getY();
            center.z = leaf_it.getZ();

            _occupied_nodes_msg.markers[marker_idx].points.push_back(center);
        }
    }
}

void get_free_nodes_msg(octomap::OcTree* _octree, visualization_msgs::MarkerArray& _free_nodes_msg, const std::string& _fixed_frame_id)
{
    // Initialize a visualization message (visualization_msgs::MarkerArray)
    ros::Time timestamp = ros::Time::now();
    _free_nodes_msg.markers.resize(_octree->getTreeDepth()+1);
    for(unsigned int i = 0; i < _free_nodes_msg.markers.size(); i++) {
        double size = _octree->getNodeSize(i);

        _free_nodes_msg.markers[i].header.frame_id = _fixed_frame_id;
        _free_nodes_msg.markers[i].header.stamp = timestamp;
        _free_nodes_msg.markers[i].id = i;
        _free_nodes_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        _free_nodes_msg.markers[i].scale.x = size;
        _free_nodes_msg.markers[i].scale.y = size;
        _free_nodes_msg.markers[i].scale.z = size;

        _free_nodes_msg.markers[i].color.r = 0.0;
        _free_nodes_msg.markers[i].color.g = 1.0;
        _free_nodes_msg.markers[i].color.b = 0.0;
        _free_nodes_msg.markers[i].color.a = 0.7;

        _free_nodes_msg.markers[i].action = visualization_msgs::Marker::ADD;
    }

    // Insert data of occupied nodes to the message
    for(auto leaf_it = _octree->begin_leafs(); leaf_it != _octree->end_leafs(); leaf_it++) {
        if(_octree->isNodeAtThreshold(*leaf_it) && !_octree->isNodeOccupied(*leaf_it)) {
            unsigned int marker_idx = leaf_it.getDepth();

            geometry_msgs::Point center;
            center.x = leaf_it.getX();
            center.y = leaf_it.getY();
            center.z = leaf_it.getZ();

            _free_nodes_msg.markers[marker_idx].points.push_back(center);
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

std_msgs::ColorRGBA get_depth_color(const int _depth, const int _MAX_DEPTH, const int _MIN_DEPTH)
{
    double depth_color = std::min(std::max((double)(_depth - _MIN_DEPTH)/ (_MAX_DEPTH - _MIN_DEPTH), 0.0), 1.0);

    // Color map: red-to-blue
    std_msgs::ColorRGBA color;
    color.r = (float)(1.0 - depth_color);
    color.g = 0.0;
    color.b = (float)depth_color;
    color.a = 1.0;

    return color;
}