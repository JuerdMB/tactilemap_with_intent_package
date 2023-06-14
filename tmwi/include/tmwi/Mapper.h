#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <tf/transform_listener.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>
#include <std_msgs/Bool.h>

// Layers for map, currently only 1 layer
const std::string STATICLAYER = "static";
const std::vector<std::string> basicLayers = {STATICLAYER};

// Screen settings
const double SCREEN_ASPECT_RATIO = 1.5;
const int SCREEN_WIDTH = 60;
const int SCREEN_HEIGHT = 40;
const double DEFAULT_ZOOM = 3.;

// Subscribed topics
const std::string PARAM_ZOOM_LEVEL = "/tmwi_node/zoom_level";

// Publish topics
const std::string TOPIC_INPUT_MAP = "/rtabmap/grid_map";
const std::string TOPIC_DOTPAD_DATA = "/mapper/output_data";
const std::string TOPIC_SCREEN_FINISHED = "/dotpad/screen_finished";

// Publish previews
const std::string TOPIC_IMAGE_SCREENSIZED = "/mapper/preview_image";
const std::string TOPIC_MAP_SCREENSIZED = "/mapper/screensized_map";
const std::string TOPIC_IMAGE_DETAILED = "/mapper/detailed_image";
const std::string TOPIC_MAP_DETAILED = "/mapper/transformed_detailed_map";

const float contrast = .9;

typedef enum {
    TRANSFORM_ERROR_NONE,
    NO_MAP_RECEIVED,
    SUBMAP_ERROR,
    ZOOM_LOOKUP_ERROR,
    ODOM_LOOKUP_ERROR
 } TRANSFORM_ERROR;

class Mapper
{
public:
    Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map);
    ~Mapper();

    void publishTransformedZoomedMap();
    TRANSFORM_ERROR getTransformedZoomedMap(grid_map::GridMap& transformedMap);

    // Params
    double zoom_level;

    // Static member functions
    static cv_bridge::CvImage get_image_from_map(grid_map::GridMap &input_map, const std::string &layer);
    static std::vector<uint8_t> getDataArray(grid_map::GridMap &inputMap, const std::string &layer);
    static void mapToScreenResolution(grid_map::GridMap &inputMap, grid_map::GridMap &outputMap);
    static void print_output_data(std::vector<uint8_t> &data);

private:
    // Basic ROS requirements
    ros::NodeHandle nodeHandle_;
    
    // Listeners
    ros::Subscriber sub_rtabmap_;
    tf::TransformListener odom_listener_;

    // Publishers
    ros::Publisher pub_bytes_dotpad;
    ros::Publisher pub_image_screensized;
    ros::Publisher pub_image_highres;

    // Getting the map from RTABMAP server
    void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);     // Incoming map from RTABMAP callback
    grid_map::GridMap globalMap_;                                       // Full map that will contain all layers, global frame
    bool receivedMap;

};