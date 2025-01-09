#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <superray_quadmap/SuperRayQuadTree.h>
#include <superray_quadmap/CullingRegionQuadTree.h>

#include <visualization_msgs/msg/marker_array.hpp>


class QuadMapVisualizer : public rclcpp::Node {
public:
    QuadMapVisualizer(const std::string& _name = "quadmap_visualization_node") : rclcpp::Node(_name) { }

    ~QuadMapVisualizer() { }

    bool initialize(const std::string& _quadmap_filename_in = "", const std::string& _fixed_frame_id_in = "map") {
        // Initialize parameters
        declare_parameter("quadmap_filename", _quadmap_filename_in);
        declare_parameter("fixed_frame_id", _fixed_frame_id_in);
        const std::string quadmap_filename = get_parameter("quadmap_filename").as_string();
        const std::string fixed_frame_id = get_parameter("fixed_frame_id").as_string();

        // Load the quadtree map
        quadmap::QuadTree* quadtree = load_map(quadmap_filename);
        if(quadtree == nullptr)
            return false;
        else
            RCLCPP_INFO_STREAM(get_logger(), "QuadMap filename: " << quadmap_filename);
    
        // Initialize the visualization data
        int num_of_leaf_nodes = init_occupancy_grid_msg(quadtree, fixed_frame_id);
        int num_of_occupieds_nodes = init_occupied_nodes_msg(quadtree, fixed_frame_id);
        int num_of_free_nodes = init_free_nodes_msg(quadtree, fixed_frame_id);
        RCLCPP_INFO_STREAM(get_logger(), "  Leaf nodes: " << num_of_leaf_nodes);
        RCLCPP_INFO_STREAM(get_logger(), "  Occupied nodes: " << num_of_occupieds_nodes);
        RCLCPP_INFO_STREAM(get_logger(), "  Free nodes: " << num_of_free_nodes);

        if(num_of_leaf_nodes != (int)quadtree->getNumLeafNodes())
            RCLCPP_WARN_STREAM(get_logger(), "Incorrect num of leaf nodes: " << num_of_leaf_nodes << " != " << (int)quadtree->getNumLeafNodes());
        if(num_of_occupieds_nodes == 0)
            RCLCPP_WARN_STREAM(get_logger(), "No occupied nodes");
        if(num_of_free_nodes == 0)
            RCLCPP_WARN_STREAM(get_logger(), "No free nodes");

        // Visualization Publishers
        occupancy_grid_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("/superray/quadmap/occupancy_grid", 1);
        occupied_nodes_publisher = create_publisher<visualization_msgs::msg::MarkerArray>("/superray/quadmap/occupied_nodes", 1);
        free_nodes_publisher     = create_publisher<visualization_msgs::msg::MarkerArray>("/superray/quadmap/free_nodes", 1);

        // Visualization Timer: 1 second
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&QuadMapVisualizer::vis_timer_callback, this));

        delete quadtree;

        return true;
    }

protected:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupancy_grid_publisher = nullptr;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_nodes_publisher = nullptr;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr free_nodes_publisher = nullptr;
    rclcpp::TimerBase::SharedPtr timer = nullptr;

    visualization_msgs::msg::MarkerArray occupancy_grid_msg;
    visualization_msgs::msg::MarkerArray occupied_nodes_msg;
    visualization_msgs::msg::MarkerArray free_nodes_msg;

    // NOTE: Marker uses this small value to draw a square since it requires a volumetric visualization.
    const double VIS_BOX_THICKNESS = 0.001;

    void vis_timer_callback() {
        if(occupancy_grid_publisher->get_subscription_count() > 0)
            occupancy_grid_publisher->publish(occupancy_grid_msg);
        if(occupied_nodes_publisher->get_subscription_count() > 0)
            occupied_nodes_publisher->publish(occupied_nodes_msg);
        if(free_nodes_publisher->get_subscription_count() > 0)
            free_nodes_publisher->publish(free_nodes_msg);
    }

    /*
    *  This function loads the quadmap from an input file.
    *  The file should have the ".bt2" or ".ot2" extention.
    */
    quadmap::QuadTree* load_map(const std::string& filename) {
        // Check the file extension
        std::string file_extension = filename.substr(filename.length() - 4, filename.length());
        if(file_extension != ".bt2" && file_extension != ".ot2") {
            RCLCPP_ERROR_STREAM(get_logger(), "Incorrect file extension: File extensions of quadmap file should be \".bt2\" or \".ot2\"");
            return nullptr;
        }

        // Load a quadmap
        quadmap::QuadTree* quadtree = new quadmap::QuadTree(10e-5);
        if(file_extension == ".bt2")
            quadtree->readBinary(filename);
        else if(file_extension == ".ot2") {
            quadmap::AbstractQuadTree* tree = quadmap::AbstractQuadTree::read(filename);
            if(tree->getTreeType() == "SuperRayQuadTree")
                quadtree = (quadmap::QuadTree*)dynamic_cast<quadmap::SuperRayQuadTree*>(tree);
            else if(tree->getTreeType() == "CullingRegionQuadTree")
                quadtree = (quadmap::QuadTree*)dynamic_cast<quadmap::CullingRegionQuadTree*>(tree);
        }

        if(quadtree->getNumLeafNodes() <= 0) {
            RCLCPP_ERROR_STREAM(get_logger(), "No quadtree node");
            return nullptr;
        }

        return quadtree;
    }

    /*
    *  This function generates a visualization message using the occupancy probabilities of all nodes of QuadMap.
    *  RViz visualizes this message as a set of the gray-scale cubes, black(occupied, 1.0) -- white(free, 0.0),
    *  with various sizes due to tree structure of QuadMap.
    */
    int init_occupancy_grid_msg(const quadmap::QuadTree* _quadtree, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::msg::MarkerArray)
        occupancy_grid_msg.markers.resize(_quadtree->getTreeDepth()+1);
        for(unsigned int i = 0; i < occupancy_grid_msg.markers.size(); i++) {
            double size = _quadtree->getNodeSize(i);

            occupancy_grid_msg.markers[i].header.frame_id = _fixed_frame_id;
            occupancy_grid_msg.markers[i].header.stamp = get_clock()->now();
            occupancy_grid_msg.markers[i].id = i;
            occupancy_grid_msg.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
            occupancy_grid_msg.markers[i].scale.x = size;
            occupancy_grid_msg.markers[i].scale.y = size;
            occupancy_grid_msg.markers[i].scale.z = VIS_BOX_THICKNESS;

            occupancy_grid_msg.markers[i].action = visualization_msgs::msg::Marker::ADD;
        }

        // Insert occupancy data to the message
        for(auto leaf_it = _quadtree->begin_leafs(); leaf_it != _quadtree->end_leafs(); leaf_it++) {
            unsigned int marker_idx = leaf_it.getDepth();

            geometry_msgs::msg::Point center;
            center.x = leaf_it.getX();
            center.y = leaf_it.getY();
            center.z = 0.0;

            std_msgs::msg::ColorRGBA color = get_occupancy_color(leaf_it->getOccupancy());

            occupancy_grid_msg.markers[marker_idx].points.push_back(center);
            occupancy_grid_msg.markers[marker_idx].colors.push_back(color);
        }

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
    int init_occupied_nodes_msg(const quadmap::QuadTree* _quadtree, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::msg::MarkerArray)
        occupied_nodes_msg.markers.resize(_quadtree->getTreeDepth()+1);
        for(unsigned int i = 0; i < occupied_nodes_msg.markers.size(); i++) {
            double size = _quadtree->getNodeSize(i);

            occupied_nodes_msg.markers[i].header.frame_id = _fixed_frame_id;
            occupied_nodes_msg.markers[i].header.stamp = get_clock()->now();
            occupied_nodes_msg.markers[i].id = i;
            occupied_nodes_msg.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
            occupied_nodes_msg.markers[i].scale.x = size;
            occupied_nodes_msg.markers[i].scale.y = size;
            occupied_nodes_msg.markers[i].scale.z = VIS_BOX_THICKNESS;

            occupied_nodes_msg.markers[i].color = get_depth_color(i, (int)occupied_nodes_msg.markers.size(), 0);

            occupied_nodes_msg.markers[i].action = visualization_msgs::msg::Marker::ADD;
        }

        // Insert data of occupied nodes to the message
        for(auto leaf_it = _quadtree->begin_leafs(); leaf_it != _quadtree->end_leafs(); leaf_it++) {
            if(_quadtree->isNodeOccupied(*leaf_it)) {
                unsigned int marker_idx = leaf_it.getDepth();

                geometry_msgs::msg::Point center;
                center.x = leaf_it.getX();
                center.y = leaf_it.getY();
                center.z = 0.0;

                occupied_nodes_msg.markers[marker_idx].points.push_back(center);
            }
        }

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
    int init_free_nodes_msg(const quadmap::QuadTree* _quadtree, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::msg::MarkerArray)
        free_nodes_msg.markers.resize(_quadtree->getTreeDepth()+1);
        for(unsigned int i = 0; i < free_nodes_msg.markers.size(); i++) {
            double size = _quadtree->getNodeSize(i);

            free_nodes_msg.markers[i].header.frame_id = _fixed_frame_id;
            free_nodes_msg.markers[i].header.stamp = get_clock()->now();
            free_nodes_msg.markers[i].id = i;
            free_nodes_msg.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
            free_nodes_msg.markers[i].scale.x = size;
            free_nodes_msg.markers[i].scale.y = size;
            free_nodes_msg.markers[i].scale.z = VIS_BOX_THICKNESS;

            free_nodes_msg.markers[i].color.r = 0.0;
            free_nodes_msg.markers[i].color.g = 1.0;
            free_nodes_msg.markers[i].color.b = 0.0;
            free_nodes_msg.markers[i].color.a = 0.7;

            free_nodes_msg.markers[i].action = visualization_msgs::msg::Marker::ADD;
        }

        // Insert data of occupied nodes to the message
        for(auto leaf_it = _quadtree->begin_leafs(); leaf_it != _quadtree->end_leafs(); leaf_it++) {
            if(_quadtree->isNodeAtThreshold(*leaf_it) && !_quadtree->isNodeOccupied(*leaf_it)) {
                unsigned int marker_idx = leaf_it.getDepth();

                geometry_msgs::msg::Point center;
                center.x = leaf_it.getX();
                center.y = leaf_it.getY();
                center.z = 0.0;

                free_nodes_msg.markers[marker_idx].points.push_back(center);
            }
        }

        // Count the visualization points
        int num_of_nodes = 0;
        for(const auto& marker : free_nodes_msg.markers)
            num_of_nodes += marker.points.size();
        return num_of_nodes;
    }

    // Function that returns a color according to given different values
    inline std_msgs::msg::ColorRGBA get_occupancy_color(const double _occupancy) {
        double occupancy_color = 1.0 - std::min(std::max(_occupancy, 0.0), 1.0);
        // Color map: gray-scale
        std_msgs::msg::ColorRGBA color;
        color.r = (float)occupancy_color;
        color.g = (float)occupancy_color;
        color.b = (float)occupancy_color;
        color.a = 1.0f;
        return color;
    }

    inline std_msgs::msg::ColorRGBA get_depth_color(const int _depth, const int _MAX_DEPTH, const int _MIN_DEPTH) {
        double depth_color = std::min(std::max((double)(_depth - _MIN_DEPTH)/ (_MAX_DEPTH - _MIN_DEPTH), 0.0), 1.0);
        // Color map: red-to-blue
        std_msgs::msg::ColorRGBA color;
        color.r = (float)(1.0 - depth_color);
        color.g = 0.0f;
        color.b = (float)depth_color;
        color.a = 1.0f;
        return color;
    }
};


void print_usage() {
    std::cout << "Usage: ros2 run occupancy_map_visualizer quadmap_visualizer_node <INPUT_FILENAME> <FIXED_FRAME_ID(optional)>" << std::endl;
    std::cout << "  <INPUT_FILENAME>  Set the input quadmap filename [.bt2|.ot2]" << std::endl;
    std::cout << "  <FIXED_FRAME_ID>  Set the fixed frame ID for visualization (default: map)" << std::endl;
}

int main(int argc, char** argv)
{
    // Run the ROS node
    rclcpp::init(argc, argv);

    std::shared_ptr<QuadMapVisualizer> quadmap_visualization_node = std::make_shared<QuadMapVisualizer>("quadmap_visualization_node");

    const std::string QUADMAP_FILENAME_IN = argc >= 2 ? argv[1] : "";
    const std::string FIXED_FRAME_ID_IN = argc >= 3 ? argv[2] : "map";
    bool success = quadmap_visualization_node->initialize(QUADMAP_FILENAME_IN, FIXED_FRAME_ID_IN);
    if(success)
        rclcpp::spin(quadmap_visualization_node);
    else
        print_usage();

    rclcpp::shutdown();

    return 0;
}