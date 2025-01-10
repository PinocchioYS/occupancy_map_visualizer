#include <ros/ros.h>
#include <chrono>

#include <superray_octomap/SuperRayOcTree.h>
#include <superray_octomap/CullingRegionOcTree.h>

#include <visualization_msgs/MarkerArray.h>


class OctoMapVisualizer {
public:
    OctoMapVisualizer() { }
    ~OctoMapVisualizer() { }

    bool initialize(const std::string& _octomap_filename_in = "", const std::string& _fixed_frame_id_in = "map") {
        // Initialize parameters
        std::string octomap_filename, fixed_frame_id;
        if(!nh.getParam("octomap_filename", octomap_filename))
            octomap_filename = _octomap_filename_in;
        if(!nh.getParam("fixed_frame_id", fixed_frame_id))
            fixed_frame_id = _fixed_frame_id_in;

        // Load the octree map
        octomap::OcTree* octree = load_map(octomap_filename);
        if(octree == nullptr)
            return false;
        else
            ROS_INFO_STREAM("OctoMap filename: " << octomap_filename);
    
        // Initialize the visualization data
        int num_of_leaf_nodes = init_occupancy_grid_msg(octree, fixed_frame_id);
        int num_of_occupieds_nodes = init_occupied_nodes_msg(octree, fixed_frame_id);
        int num_of_free_nodes = init_free_nodes_msg(octree, fixed_frame_id);
        ROS_INFO_STREAM("  Leaf nodes: " << num_of_leaf_nodes);
        ROS_INFO_STREAM("  Occupied nodes: " << num_of_occupieds_nodes);
        ROS_INFO_STREAM("  Free nodes: " << num_of_free_nodes);

        if(num_of_leaf_nodes != (int)octree->getNumLeafNodes())
            ROS_WARN_STREAM("Incorrect num of leaf nodes: " << num_of_leaf_nodes << " != " << (int)octree->getNumLeafNodes());
        if(num_of_occupieds_nodes == 0)
            ROS_WARN_STREAM("No occupied nodes");
        if(num_of_free_nodes == 0)
            ROS_WARN_STREAM("No free nodes");

        // Visualization Publishers
        occupancy_grid_publisher = nh.advertise<visualization_msgs::MarkerArray>("/superray/octomap/occupancy_grid", 1);
        occupied_nodes_publisher = nh.advertise<visualization_msgs::MarkerArray>("/superray/octomap/occupied_nodes", 1);
        free_nodes_publisher     = nh.advertise<visualization_msgs::MarkerArray>("/superray/octomap/free_nodes", 1);

        // Visualization Timer: 1 second
        timer = nh.createTimer(ros::Duration(1), &OctoMapVisualizer::vis_timer_callback, this);

        delete octree;

        return true;
    }

protected:
    ros::NodeHandle nh;

    ros::Publisher occupancy_grid_publisher;
    ros::Publisher occupied_nodes_publisher;
    ros::Publisher free_nodes_publisher;
    ros::Timer timer;

    visualization_msgs::MarkerArray occupancy_grid_msg;
    visualization_msgs::MarkerArray occupied_nodes_msg;
    visualization_msgs::MarkerArray free_nodes_msg;

    void vis_timer_callback(const ros::TimerEvent& event) {
        if(occupancy_grid_publisher.getNumSubscribers() > 0)
            occupancy_grid_publisher.publish(occupancy_grid_msg);
        if(occupied_nodes_publisher.getNumSubscribers() > 0)
            occupied_nodes_publisher.publish(occupied_nodes_msg);
        if(free_nodes_publisher.getNumSubscribers() > 0)
            free_nodes_publisher.publish(free_nodes_msg);
    }

    /*
    *  This function loads the octomap from an input file.
    *  The file should have the ".bt" or ".ot" extention.
    */
    octomap::OcTree* load_map(const std::string& filename) {
        // Check the file extension
        std::string file_extension = filename.substr(filename.length() - 3, filename.length());
        if(file_extension != ".bt" && file_extension != ".ot") {
            ROS_ERROR_STREAM("Incorrect file extension: File extensions of octomap file should be \".bt\" or \".ot\"");
            return nullptr;
        }

        // Load an octomap
        octomap::OcTree* octree = new octomap::OcTree(10e-5);
        if(file_extension == ".bt")
            octree->readBinary(filename);
        else if(file_extension == ".ot") {
            octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
            if(tree->getTreeType() == "SuperRayOcTree")
                octree = (octomap::OcTree*)dynamic_cast<octomap::SuperRayOcTree*>(tree);
            else if(tree->getTreeType() == "CullingRegionOcTree")
                octree = (octomap::OcTree*)dynamic_cast<octomap::CullingRegionOcTree*>(tree);
        }

        if(octree->getNumLeafNodes() <= 0) {
            ROS_ERROR_STREAM("No octree node");
            return nullptr;
        }

        return octree;
    }

    /*
    *  This function generates a visualization message using the occupancy probabilities of all nodes of OctoMap.
    *  RViz visualizes this message as a set of the gray-scale cubes, black(occupied, 1.0) -- white(free, 0.0),
    *  with various sizes due to tree structure of OctoMap.
    */
    int init_occupancy_grid_msg(const octomap::OcTree* _octree, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::MarkerArray)
        occupancy_grid_msg.markers.resize(_octree->getTreeDepth()+1);
        for(unsigned int i = 0; i < occupancy_grid_msg.markers.size(); i++) {
            double size = _octree->getNodeSize(i);

            occupancy_grid_msg.markers[i].header.frame_id = _fixed_frame_id;
            occupancy_grid_msg.markers[i].header.stamp = ros::Time::now();
            occupancy_grid_msg.markers[i].id = i;
            occupancy_grid_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupancy_grid_msg.markers[i].scale.x = size;
            occupancy_grid_msg.markers[i].scale.y = size;
            occupancy_grid_msg.markers[i].scale.z = size;

            occupancy_grid_msg.markers[i].pose.position.x = 0.0;
            occupancy_grid_msg.markers[i].pose.position.y = 0.0;
            occupancy_grid_msg.markers[i].pose.position.z = 0.0;
            occupancy_grid_msg.markers[i].pose.orientation.x = 0.0;
            occupancy_grid_msg.markers[i].pose.orientation.y = 0.0;
            occupancy_grid_msg.markers[i].pose.orientation.z = 0.0;
            occupancy_grid_msg.markers[i].pose.orientation.w = 1.0;

            occupancy_grid_msg.markers[i].action = visualization_msgs::Marker::ADD;
        }

        // Insert occupancy data to the message
        for(auto leaf_it = _octree->begin_leafs(); leaf_it != _octree->end_leafs(); leaf_it++) {
            unsigned int marker_idx = leaf_it.getDepth();

            geometry_msgs::Point center;
            center.x = leaf_it.getX();
            center.y = leaf_it.getY();
            center.z = leaf_it.getZ();

            std_msgs::ColorRGBA color = get_occupancy_color(leaf_it->getOccupancy());

            occupancy_grid_msg.markers[marker_idx].points.push_back(center);
            occupancy_grid_msg.markers[marker_idx].colors.push_back(color);
        }

        // Remove empty markers: turn-off error/warning messages on RViz
        occupancy_grid_msg.markers.erase(
            std::remove_if(occupancy_grid_msg.markers.begin(), occupancy_grid_msg.markers.end(), [](const auto& marker) { return marker.points.empty(); }), 
            occupancy_grid_msg.markers.end()
        );

        // Count the visualization points
        int num_of_nodes = 0;
        for(const auto& marker : occupancy_grid_msg.markers)
            num_of_nodes += marker.points.size();
        return num_of_nodes;
    }

    /*
    *  This function generates a visualization message using the nodes classified to occupied state.
    *  RViz shows the occupied nodes with various colors depending on depth levels; red(root) -- blue(max_depth).
    */
    int init_occupied_nodes_msg(const octomap::OcTree* _octree, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::MarkerArray)
        occupied_nodes_msg.markers.resize(_octree->getTreeDepth()+1);
        for(unsigned int i = 0; i < occupied_nodes_msg.markers.size(); i++) {
            double size = _octree->getNodeSize(i);

            occupied_nodes_msg.markers[i].header.frame_id = _fixed_frame_id;
            occupied_nodes_msg.markers[i].header.stamp = ros::Time::now();
            occupied_nodes_msg.markers[i].id = i;
            occupied_nodes_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupied_nodes_msg.markers[i].scale.x = size;
            occupied_nodes_msg.markers[i].scale.y = size;
            occupied_nodes_msg.markers[i].scale.z = size;

            occupied_nodes_msg.markers[i].pose.position.x = 0.0;
            occupied_nodes_msg.markers[i].pose.position.y = 0.0;
            occupied_nodes_msg.markers[i].pose.position.z = 0.0;
            occupied_nodes_msg.markers[i].pose.orientation.x = 0.0;
            occupied_nodes_msg.markers[i].pose.orientation.y = 0.0;
            occupied_nodes_msg.markers[i].pose.orientation.z = 0.0;
            occupied_nodes_msg.markers[i].pose.orientation.w = 1.0;

            occupied_nodes_msg.markers[i].color = get_depth_color(i, (int)occupied_nodes_msg.markers.size(), 0);

            occupied_nodes_msg.markers[i].action = visualization_msgs::Marker::ADD;
        }

        // Insert data of occupied nodes to the message
        for(auto leaf_it = _octree->begin_leafs(); leaf_it != _octree->end_leafs(); leaf_it++) {
            if(_octree->isNodeOccupied(*leaf_it)) {
                unsigned int marker_idx = leaf_it.getDepth();

                geometry_msgs::Point center;
                center.x = leaf_it.getX();
                center.y = leaf_it.getY();
                center.z = leaf_it.getZ();

                occupied_nodes_msg.markers[marker_idx].points.push_back(center);
            }
        }

        // Remove empty markers: turn-off error/warning messages on RViz
        occupied_nodes_msg.markers.erase(
            std::remove_if(occupied_nodes_msg.markers.begin(), occupied_nodes_msg.markers.end(), [](const auto& marker) { return marker.points.empty(); }), 
            occupied_nodes_msg.markers.end()
        );

        // Count the visualization points
        int num_of_nodes = 0;
        for(const auto& marker : occupied_nodes_msg.markers)
            num_of_nodes += marker.points.size();
        return num_of_nodes;
    }

    /*
    *  This function generates a visualization message using the nodes classified to free state.
    *  RViz visualizes the free nodes to a set of the green cubes with different sizes.
    */
    int init_free_nodes_msg(const octomap::OcTree* _octree, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::MarkerArray)
        free_nodes_msg.markers.resize(_octree->getTreeDepth()+1);
        for(unsigned int i = 0; i < free_nodes_msg.markers.size(); i++) {
            double size = _octree->getNodeSize(i);

            free_nodes_msg.markers[i].header.frame_id = _fixed_frame_id;
            free_nodes_msg.markers[i].header.stamp = ros::Time::now();
            free_nodes_msg.markers[i].id = i;
            free_nodes_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            free_nodes_msg.markers[i].scale.x = size;
            free_nodes_msg.markers[i].scale.y = size;
            free_nodes_msg.markers[i].scale.z = size;

            free_nodes_msg.markers[i].pose.position.x = 0.0;
            free_nodes_msg.markers[i].pose.position.y = 0.0;
            free_nodes_msg.markers[i].pose.position.z = 0.0;
            free_nodes_msg.markers[i].pose.orientation.x = 0.0;
            free_nodes_msg.markers[i].pose.orientation.y = 0.0;
            free_nodes_msg.markers[i].pose.orientation.z = 0.0;
            free_nodes_msg.markers[i].pose.orientation.w = 1.0;

            free_nodes_msg.markers[i].color.r = 0.0;
            free_nodes_msg.markers[i].color.g = 1.0;
            free_nodes_msg.markers[i].color.b = 0.0;
            free_nodes_msg.markers[i].color.a = 0.7;

            free_nodes_msg.markers[i].action = visualization_msgs::Marker::ADD;
        }

        // Insert data of occupied nodes to the message
        for(auto leaf_it = _octree->begin_leafs(); leaf_it != _octree->end_leafs(); leaf_it++) {
            if(_octree->isNodeAtThreshold(*leaf_it) && !_octree->isNodeOccupied(*leaf_it)) {
                unsigned int marker_idx = leaf_it.getDepth();

                geometry_msgs::Point center;
                center.x = leaf_it.getX();
                center.y = leaf_it.getY();
                center.z = leaf_it.getZ();

                free_nodes_msg.markers[marker_idx].points.push_back(center);
            }
        }

        // Remove empty markers: turn-off error/warning messages on RViz
        free_nodes_msg.markers.erase(
            std::remove_if(free_nodes_msg.markers.begin(), free_nodes_msg.markers.end(), [](const auto& marker) { return marker.points.empty(); }), 
            free_nodes_msg.markers.end()
        );

        // Count the visualization points
        int num_of_nodes = 0;
        for(const auto& marker : free_nodes_msg.markers)
            num_of_nodes += marker.points.size();
        return num_of_nodes;
    }

    // Function that returns a color according to given different values
    inline std_msgs::ColorRGBA get_occupancy_color(const double _occupancy) {
        double occupancy_color = 1.0 - std::min(std::max(_occupancy, 0.0), 1.0);
        // Color map: gray-scale
        std_msgs::ColorRGBA color;
        color.r = (float)occupancy_color;
        color.g = (float)occupancy_color;
        color.b = (float)occupancy_color;
        color.a = 1.0f;
        return color;
    }

    inline std_msgs::ColorRGBA get_depth_color(const int _depth, const int _MAX_DEPTH, const int _MIN_DEPTH) {
        double depth_color = std::min(std::max((double)(_depth - _MIN_DEPTH)/ (_MAX_DEPTH - _MIN_DEPTH), 0.0), 1.0);
        // Color map: red-to-blue
        std_msgs::ColorRGBA color;
        color.r = (float)(1.0 - depth_color);
        color.g = 0.0f;
        color.b = (float)depth_color;
        color.a = 1.0f;
        return color;
    }
};

void print_usage() {
    std::cout << "Usage: ros2 run occupancy_map_visualizer octomap_visualizer_node <INPUT_FILENAME> <FIXED_FRAME_ID(optional)>" << std::endl;
    std::cout << "  <INPUT_FILENAME>  Set the input octomap filename [.bt|.ot]" << std::endl;
    std::cout << "  <FIXED_FRAME_ID>  Set the fixed frame ID for visualization (default: map)" << std::endl;
}

int main(int argc, char** argv)
{
    // Run the ROS node
    ros::init(argc, argv, "octomap_visualizer_node");

    std::shared_ptr<OctoMapVisualizer> octomap_visualization_node = std::make_shared<OctoMapVisualizer>();

    const std::string OCTOMAP_FILENAME_IN = argc >= 2 ? argv[1] : "";
    const std::string FIXED_FRAME_ID_IN = argc >= 3 ? argv[2] : "map";
    bool success = octomap_visualization_node->initialize(OCTOMAP_FILENAME_IN, FIXED_FRAME_ID_IN);
    if(success)
        ros::spin();
    else
        print_usage();

    ros::shutdown();

    return 0;
}