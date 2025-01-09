#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <superray_gridmap2d/SuperRayGrid2D.h>
#include <superray_gridmap2d/CullingRegionGrid2D.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>


class GridMap2DVisualizer : public rclcpp::Node {
public:
    GridMap2DVisualizer(const std::string& _name = "gridmap2d_visualization_node") : rclcpp::Node(_name) { }

    ~GridMap2DVisualizer() { }

    bool initialize(const std::string& _gridmap2d_filename_in = "", const std::string& _fixed_frame_id_in = "map") {
        // Initialize parameters
        declare_parameter("gridmap2d_filename", _gridmap2d_filename_in);
        declare_parameter("fixed_frame_id", _fixed_frame_id_in);
        const std::string gridmap2d_filename = get_parameter("gridmap2d_filename").as_string();
        const std::string fixed_frame_id = get_parameter("fixed_frame_id").as_string();

        // Load the grid map
        gridmap2d::Grid2D* grid2d = load_map(gridmap2d_filename);
        if(grid2d == nullptr)
            return false;
        else
            RCLCPP_INFO_STREAM(get_logger(), "GridMap2D filename: " << gridmap2d_filename);
    
        // Initialize the visualization data
        int num_of_nodes = init_occupancy_grid_msg(grid2d, fixed_frame_id);
        int num_of_occupieds_nodes = init_occupied_nodes_msg(grid2d, fixed_frame_id);
        int num_of_free_nodes = init_free_nodes_msg(grid2d, fixed_frame_id);
        RCLCPP_INFO_STREAM(get_logger(), "  Nodes: " << num_of_nodes << " (including unknown cells)");
        RCLCPP_INFO_STREAM(get_logger(), "  Occupied nodes: " << num_of_occupieds_nodes);
        RCLCPP_INFO_STREAM(get_logger(), "  Free nodes: " << num_of_free_nodes);

        if(num_of_nodes < (int)grid2d->size())
            RCLCPP_WARN_STREAM(get_logger(), "Incorrect num of nodes: " << num_of_nodes << " < " << (int)grid2d->size());
        if(num_of_occupieds_nodes == 0)
            RCLCPP_WARN_STREAM(get_logger(), "No occupied nodes");
        if(num_of_free_nodes == 0)
            RCLCPP_WARN_STREAM(get_logger(), "No free nodes");

        // Visualization Publishers
        occupancy_grid_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/superray/gridmap2d/occupancy_grid", 1);
        occupied_nodes_publisher = create_publisher<visualization_msgs::msg::Marker>("/superray/gridmap2d/occupied_nodes", 1);
        free_nodes_publisher     = create_publisher<visualization_msgs::msg::Marker>("/superray/gridmap2d/free_nodes", 1);

        // Visualization Timer: 1 second
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&GridMap2DVisualizer::vis_timer_callback, this));

        delete grid2d;

        return true;
    }

protected:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher = nullptr;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_nodes_publisher = nullptr;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr free_nodes_publisher = nullptr;
    rclcpp::TimerBase::SharedPtr timer = nullptr;

    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    visualization_msgs::msg::Marker occupied_nodes_msg;
    visualization_msgs::msg::Marker free_nodes_msg;

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
     *  This function loads the gridmap2d from an input file.
     *  The file should have the ".bg2" or ".og2" extention.
     */
    gridmap2d::Grid2D* load_map(const std::string& filename) {
        // Check the file extension
        std::string file_extension = filename.substr(filename.length() - 4, filename.length());
        if(file_extension != ".bg2" && file_extension != ".og2") {
            RCLCPP_ERROR_STREAM(get_logger(), "Incorrect file extension: File extensions of gridmap2d file should be \".bg2\" or \".og2\"");
            return nullptr;
        }

        // Load a gridmap2d
        gridmap2d::Grid2D* grid2d = new gridmap2d::Grid2D(10e-5);
        if(file_extension == ".bg2")
            grid2d->readBinary(filename);
        else if(file_extension == ".og2") {
            gridmap2d::AbstractGrid2D* grid = gridmap2d::AbstractGrid2D::read(filename);
            if(grid->getGridType() == "SuperRayGrid2D")
                grid2d = (gridmap2d::Grid2D*)dynamic_cast<gridmap2d::SuperRayGrid2D*>(grid);
            else if(grid->getGridType() == "CullingRegionGrid2D")
                grid2d = (gridmap2d::Grid2D*)dynamic_cast<gridmap2d::CullingRegionGrid2D*>(grid);
        }

        if(grid2d->size() <= 0) {
            RCLCPP_ERROR_STREAM(get_logger(), "No grid2d node");
            return nullptr;
        }

        return grid2d;
    }

    /*
     *  This function generates a visualization message using the occupancy probabilities of all nodes of GridMap2D.
     *  RViz visualizes this message as a set of the gray-scale cubes, black(occupied, 1.0) -- white(free, 0.0).
     */
    int init_occupancy_grid_msg(const gridmap2d::Grid2D* _grid2d, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (nav_msgs::msg::OccupancyGrid)
        double min_x, min_y, max_x, max_y;
        _grid2d->getMetricMin(min_x, min_y);
        _grid2d->getMetricMax(max_x, max_y);
        gridmap2d::Grid2DKey min_key = _grid2d->coordToKey(min_x, min_y);
        gridmap2d::Grid2DKey max_key = _grid2d->coordToKey(max_x, max_y);

        // Initialize a visualization message (nav_msgs::OccupancyGrid)
        occupancy_grid_msg.header.frame_id = _fixed_frame_id;
        occupancy_grid_msg.header.stamp = get_clock()->now();

        occupancy_grid_msg.info.resolution = (float)_grid2d->getResolution();

        occupancy_grid_msg.info.origin.position.x = _grid2d->keyToCoord(min_key[0]) - (_grid2d->getResolution() / 2.0);
        occupancy_grid_msg.info.origin.position.y = _grid2d->keyToCoord(min_key[1]) - (_grid2d->getResolution() / 2.0);
        occupancy_grid_msg.info.origin.position.z = 0.0;
        occupancy_grid_msg.info.origin.orientation.x = 0.0;
        occupancy_grid_msg.info.origin.orientation.y = 0.0;
        occupancy_grid_msg.info.origin.orientation.z = 0.0;
        occupancy_grid_msg.info.origin.orientation.w = 1.0;

        occupancy_grid_msg.info.width = (unsigned int)(max_key[0] - min_key[0] + 1);
        occupancy_grid_msg.info.height = (unsigned int)(max_key[1] - min_key[1] + 1);
        occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, -1);

        // Insert occupancy data to the message
        for(auto it = _grid2d->getGrid()->begin(); it != _grid2d->getGrid()->end(); it++){
            gridmap2d::Grid2DKey node_key = it->first;
            int cell_x = node_key[0] - min_key[0];
            int cell_y = node_key[1] - min_key[1];
            int cell_idx = cell_y * occupancy_grid_msg.info.width + cell_x;

            occupancy_grid_msg.data[cell_idx] = (char)(it->second->getOccupancy() * 100.0);
        }

        // NOTE: the OccupancyGrid includes unknown cells
        return occupancy_grid_msg.info.width * occupancy_grid_msg.info.height;
    }

    /*
     *  This function generates a visualization message using the nodes classified to occupied state.
     *  RViz shows the occupied nodes as a set of the blue-colored voxels.
     */
    int init_occupied_nodes_msg(const gridmap2d::Grid2D* _grid2d, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::msg::Marker)
        double size = _grid2d->getResolution();

        occupied_nodes_msg.header.frame_id = _fixed_frame_id;
        occupied_nodes_msg.header.stamp = get_clock()->now();
        occupied_nodes_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
        occupied_nodes_msg.scale.x = size;
        occupied_nodes_msg.scale.y = size;
        occupied_nodes_msg.scale.z = VIS_BOX_THICKNESS;

        occupied_nodes_msg.color.r = 0.0;
        occupied_nodes_msg.color.g = 0.0;
        occupied_nodes_msg.color.b = 1.0;
        occupied_nodes_msg.color.a = 1.0;

        occupied_nodes_msg.action = visualization_msgs::msg::Marker::ADD;

        // Insert data of occupied nodes to the message
        for(auto it = _grid2d->getGrid()->begin(); it != _grid2d->getGrid()->end(); it++) {
            if(_grid2d->isNodeOccupied(it->second)) {
                geometry_msgs::msg::Point center;
                center.x = _grid2d->keyToCoord(it->first[0]);
                center.y = _grid2d->keyToCoord(it->first[1]);
                center.z = 0.0;

                occupied_nodes_msg.points.push_back(center);
            }
        }

        return occupied_nodes_msg.points.size();
    }

    /*
     *  This function generates a visualization message using the nodes classified to free state.
     *  RViz visualizes the free nodes to a set of the green voxels.
     */
    int init_free_nodes_msg(const gridmap2d::Grid2D* _grid2d, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::msg::Marker)
        double size = _grid2d->getResolution();

        free_nodes_msg.header.frame_id = _fixed_frame_id;
        free_nodes_msg.header.stamp = get_clock()->now();
        free_nodes_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
        free_nodes_msg.scale.x = size;
        free_nodes_msg.scale.y = size;
        free_nodes_msg.scale.z = VIS_BOX_THICKNESS;

        free_nodes_msg.color.r = 0.0;
        free_nodes_msg.color.g = 1.0;
        free_nodes_msg.color.b = 0.0;
        free_nodes_msg.color.a = 0.7;

        free_nodes_msg.action = visualization_msgs::msg::Marker::ADD;

        // Insert data of occupied nodes to the message
        for(auto it = _grid2d->getGrid()->begin(); it != _grid2d->getGrid()->end(); it++) {
            if(_grid2d->isNodeAtThreshold(it->second) && !_grid2d->isNodeOccupied(it->second)) {
                geometry_msgs::msg::Point center;
                center.x = _grid2d->keyToCoord(it->first[0]);
                center.y = _grid2d->keyToCoord(it->first[1]);
                center.z = 0.0;

                free_nodes_msg.points.push_back(center);
            }
        }

        return free_nodes_msg.points.size();
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
};

void print_usage() {
    std::cout << "Usage: ros2 run occupancy_map_visualizer gridmap2d_visualizer_node <INPUT_FILENAME> <FIXED_FRAME_ID(optional)>" << std::endl;
    std::cout << "  <INPUT_FILENAME>  Set the input gridmap2d filename [.bg2|.og2]" << std::endl;
    std::cout << "  <FIXED_FRAME_ID>  Set the fixed frame ID for visualization (default: map)" << std::endl;
}

int main(int argc, char** argv)
{
    // Run the ROS node
    rclcpp::init(argc, argv);

    std::shared_ptr<GridMap2DVisualizer> gridmap2d_visualization_node = std::make_shared<GridMap2DVisualizer>("gridmap2d_visualization_node");

    const std::string GRIDMAP2D_FILENAME_IN = argc >= 2 ? argv[1] : "";
    const std::string FIXED_FRAME_ID_IN = argc >= 3 ? argv[2] : "map";
    bool success = gridmap2d_visualization_node->initialize(GRIDMAP2D_FILENAME_IN, FIXED_FRAME_ID_IN);
    if(success)
        rclcpp::spin(gridmap2d_visualization_node);
    else
        print_usage();

    rclcpp::shutdown();

    return 0;
}