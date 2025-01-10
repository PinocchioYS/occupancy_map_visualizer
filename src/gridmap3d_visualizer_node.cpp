#include <ros/ros.h>

#include <superray_gridmap3d/SuperRayGrid3D.h>
#include <superray_gridmap3d/CullingRegionGrid3D.h>

#include <visualization_msgs/Marker.h>


class GridMap3DVisualizer {
public:
    GridMap3DVisualizer() { }
    ~GridMap3DVisualizer() { }

    bool initialize(const std::string& _gridmap3d_filename_in = "", const std::string& _fixed_frame_id_in = "map") {
        // Initialize parameters
        std::string gridmap3d_filename, fixed_frame_id;
        if(!nh.getParam("gridmap3d_filename", gridmap3d_filename))
            gridmap3d_filename = _gridmap3d_filename_in;
        if(!nh.getParam("fixed_frame_id", fixed_frame_id))
            fixed_frame_id = _fixed_frame_id_in;

        // Load the grid map
        gridmap3d::Grid3D* grid3d = load_map(gridmap3d_filename);
        if(grid3d == nullptr)
            return false;
        else
            ROS_INFO_STREAM("GridMap3D filename: " << gridmap3d_filename);
    
        // Initialize the visualization data
        int num_of_nodes = init_occupancy_grid_msg(grid3d, fixed_frame_id);
        int num_of_occupieds_nodes = init_occupied_nodes_msg(grid3d, fixed_frame_id);
        int num_of_free_nodes = init_free_nodes_msg(grid3d, fixed_frame_id);
        ROS_INFO_STREAM("  Nodes: " << num_of_nodes);
        ROS_INFO_STREAM("  Occupied nodes: " << num_of_occupieds_nodes);
        ROS_INFO_STREAM("  Free nodes: " << num_of_free_nodes);

        if(num_of_nodes != (int)grid3d->size())
            ROS_WARN_STREAM("Incorrect num of nodes: " << num_of_nodes << " != " << (int)grid3d->size());
        if(num_of_occupieds_nodes == 0)
            ROS_WARN_STREAM("No occupied nodes");
        if(num_of_free_nodes == 0)
            ROS_WARN_STREAM("No free nodes");

        // Visualization Publishers
        occupancy_grid_publisher = nh.advertise<visualization_msgs::Marker>("/superray/gridmap3d/occupancy_grid", 1);
        occupied_nodes_publisher = nh.advertise<visualization_msgs::Marker>("/superray/gridmap3d/occupied_nodes", 1);
        free_nodes_publisher     = nh.advertise<visualization_msgs::Marker>("/superray/gridmap3d/free_nodes", 1);

        // Visualization Timer: 1 second
        timer = nh.createTimer(ros::Duration(1), &GridMap3DVisualizer::vis_timer_callback, this);

        delete grid3d;

        return true;
    }

protected:
    ros::NodeHandle nh;

    ros::Publisher occupancy_grid_publisher;
    ros::Publisher occupied_nodes_publisher;
    ros::Publisher free_nodes_publisher;
    ros::Timer timer;

    visualization_msgs::Marker occupancy_grid_msg;
    visualization_msgs::Marker occupied_nodes_msg;
    visualization_msgs::Marker free_nodes_msg;

    void vis_timer_callback(const ros::TimerEvent& event) {
        if(occupancy_grid_publisher.getNumSubscribers() > 0)
            occupancy_grid_publisher.publish(occupancy_grid_msg);
        if(occupied_nodes_publisher.getNumSubscribers() > 0)
            occupied_nodes_publisher.publish(occupied_nodes_msg);
        if(free_nodes_publisher.getNumSubscribers() > 0)
            free_nodes_publisher.publish(free_nodes_msg);
    }

    /*
     *  This function loads the gridmap3d from an input file.
     *  The file should have the ".bg3" or ".og3" extention.
     */
    gridmap3d::Grid3D* load_map(const std::string& filename) {
        // Check the file extension
        std::string file_extension = filename.substr(filename.length() - 4, filename.length());
        if(file_extension != ".bg3" && file_extension != ".og3") {
            ROS_ERROR_STREAM("Incorrect file extension: File extensions of gridmap3d file should be \".bg3\" or \".og3\"");
            return nullptr;
        }

        // Load a gridmap3d
        gridmap3d::Grid3D* grid3d = new gridmap3d::Grid3D(10e-5);
        if(file_extension == ".bg3")
            grid3d->readBinary(filename);
        else if(file_extension == ".og3") {
            gridmap3d::AbstractGrid3D* grid = gridmap3d::AbstractGrid3D::read(filename);
            if(grid->getGridType() == "SuperRayGrid3D")
                grid3d = (gridmap3d::Grid3D*)dynamic_cast<gridmap3d::SuperRayGrid3D*>(grid);
            else if(grid->getGridType() == "CullingRegionGrid3D")
                grid3d = (gridmap3d::Grid3D*)dynamic_cast<gridmap3d::CullingRegionGrid3D*>(grid);
        }

        if(grid3d->size() <= 0) {
            ROS_ERROR_STREAM("No grid3d node");
            return nullptr;
        }

        return grid3d;
    }

    /*
     *  This function generates a visualization message using the occupancy probabilities of all nodes of GridMap3D.
     *  RViz visualizes this message as a set of the gray-scale cubes, black(occupied, 1.0) -- white(free, 0.0).
     */
    int init_occupancy_grid_msg(const gridmap3d::Grid3D* _grid3d, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::Marker)
        double size = _grid3d->getResolution();

        occupancy_grid_msg.header.frame_id = _fixed_frame_id;
        occupancy_grid_msg.header.stamp = ros::Time::now();
        occupancy_grid_msg.type = visualization_msgs::Marker::CUBE_LIST;
        occupancy_grid_msg.scale.x = size;
        occupancy_grid_msg.scale.y = size;
        occupancy_grid_msg.scale.z = size;

        occupancy_grid_msg.pose.position.x = 0.0;
        occupancy_grid_msg.pose.position.y = 0.0;
        occupancy_grid_msg.pose.position.z = 0.0;
        occupancy_grid_msg.pose.orientation.x = 0.0;
        occupancy_grid_msg.pose.orientation.y = 0.0;
        occupancy_grid_msg.pose.orientation.z = 0.0;
        occupancy_grid_msg.pose.orientation.w = 1.0;

        occupancy_grid_msg.action = visualization_msgs::Marker::ADD;

        // Insert occupancy data to the message
        for(auto it = _grid3d->getGrid()->begin(); it != _grid3d->getGrid()->end(); it++) {
            geometry_msgs::Point center;
            center.x = _grid3d->keyToCoord(it->first[0]);
            center.y = _grid3d->keyToCoord(it->first[1]);
            center.z = _grid3d->keyToCoord(it->first[2]);

            std_msgs::ColorRGBA color = get_occupancy_color(it->second->getOccupancy());

            occupancy_grid_msg.points.push_back(center);
            occupancy_grid_msg.colors.push_back(color);
        }

        return occupancy_grid_msg.points.size();
    }

    /*
     *  This function generates a visualization message using the nodes classified to occupied state.
     *  RViz shows the occupied nodes as a set of the blue-colored voxels.
     */
    int init_occupied_nodes_msg(const gridmap3d::Grid3D* _grid3d, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::Marker)
        double size = _grid3d->getResolution();

        occupied_nodes_msg.header.frame_id = _fixed_frame_id;
        occupied_nodes_msg.header.stamp = ros::Time::now();
        occupied_nodes_msg.type = visualization_msgs::Marker::CUBE_LIST;
        occupied_nodes_msg.scale.x = size;
        occupied_nodes_msg.scale.y = size;
        occupied_nodes_msg.scale.z = size;

        occupied_nodes_msg.pose.position.x = 0.0;
        occupied_nodes_msg.pose.position.y = 0.0;
        occupied_nodes_msg.pose.position.z = 0.0;
        occupied_nodes_msg.pose.orientation.x = 0.0;
        occupied_nodes_msg.pose.orientation.y = 0.0;
        occupied_nodes_msg.pose.orientation.z = 0.0;
        occupied_nodes_msg.pose.orientation.w = 1.0;

        occupied_nodes_msg.color.r = 0.0;
        occupied_nodes_msg.color.g = 0.0;
        occupied_nodes_msg.color.b = 1.0;
        occupied_nodes_msg.color.a = 1.0;

        occupied_nodes_msg.action = visualization_msgs::Marker::ADD;

        // Insert data of occupied nodes to the message
        for(auto it = _grid3d->getGrid()->begin(); it != _grid3d->getGrid()->end(); it++) {
            if(_grid3d->isNodeOccupied(it->second)) {
                geometry_msgs::Point center;
                center.x = _grid3d->keyToCoord(it->first[0]);
                center.y = _grid3d->keyToCoord(it->first[1]);
                center.z = _grid3d->keyToCoord(it->first[2]);

                occupied_nodes_msg.points.push_back(center);
            }
        }

        return occupied_nodes_msg.points.size();
    }

    /*
     *  This function generates a visualization message using the nodes classified to free state.
     *  RViz visualizes the free nodes to a set of the green voxels.
     */
    int init_free_nodes_msg(const gridmap3d::Grid3D* _grid3d, const std::string& _fixed_frame_id = "map") {
        // Initialize a visualization message (visualization_msgs::Marker)
        double size = _grid3d->getResolution();

        free_nodes_msg.header.frame_id = _fixed_frame_id;
        free_nodes_msg.header.stamp = ros::Time::now();
        free_nodes_msg.type = visualization_msgs::Marker::CUBE_LIST;
        free_nodes_msg.scale.x = size;
        free_nodes_msg.scale.y = size;
        free_nodes_msg.scale.z = size;

        free_nodes_msg.pose.position.x = 0.0;
        free_nodes_msg.pose.position.y = 0.0;
        free_nodes_msg.pose.position.z = 0.0;
        free_nodes_msg.pose.orientation.x = 0.0;
        free_nodes_msg.pose.orientation.y = 0.0;
        free_nodes_msg.pose.orientation.z = 0.0;
        free_nodes_msg.pose.orientation.w = 1.0;

        free_nodes_msg.color.r = 0.0;
        free_nodes_msg.color.g = 1.0;
        free_nodes_msg.color.b = 0.0;
        free_nodes_msg.color.a = 0.7;

        free_nodes_msg.action = visualization_msgs::Marker::ADD;

        // Insert data of occupied nodes to the message
        for(auto it = _grid3d->getGrid()->begin(); it != _grid3d->getGrid()->end(); it++) {
            if(_grid3d->isNodeAtThreshold(it->second) && !_grid3d->isNodeOccupied(it->second)) {
                geometry_msgs::Point center;
                center.x = _grid3d->keyToCoord(it->first[0]);
                center.y = _grid3d->keyToCoord(it->first[1]);
                center.z = _grid3d->keyToCoord(it->first[2]);

                free_nodes_msg.points.push_back(center);
            }
        }

        return free_nodes_msg.points.size();
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
};

void print_usage() {
    std::cout << "Usage: ros2 run occupancy_map_visualizer gridmap3d_visualizer_node <INPUT_FILENAME> <FIXED_FRAME_ID(optional)>" << std::endl;
    std::cout << "  <INPUT_FILENAME>  Set the input gridmap3d filename [.bg3|.og3]" << std::endl;
    std::cout << "  <FIXED_FRAME_ID>  Set the fixed frame ID for visualization (default: map)" << std::endl;
}

int main(int argc, char** argv)
{
    // Run the ROS node
    ros::init(argc, argv, "gridmap3d_visualizer_node");

    std::shared_ptr<GridMap3DVisualizer> gridmap3d_visualization_node = std::make_shared<GridMap3DVisualizer>();

    const std::string GRIDMAP3D_FILENAME_IN = argc >= 2 ? argv[1] : "";
    const std::string FIXED_FRAME_ID_IN = argc >= 3 ? argv[2] : "map";
    bool success = gridmap3d_visualization_node->initialize(GRIDMAP3D_FILENAME_IN, FIXED_FRAME_ID_IN);
    if(success)
        ros::spin();
    else
        print_usage();

    ros::shutdown();

    return 0;
}