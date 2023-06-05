#pragma once

#include <string>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>


// Layers for map, currently only 1 layer
const std::string STATICLAYER = "static";
const std::vector<std::string> basicLayers = {STATICLAYER};

#define SCRN_DEFAULT_RATE 5
#define SCRN_ASPECT_RATIO 1.5

// Zoom easing
const double ZOOM_DEFAULT = 10.;
const double ZOOMERROR_THRESHOLD = .05;
const double DAMPING_FACTOR = .6;
#define ZOOM_DEFAULT_RATE 5.
const std::string MAP_ZOOM_TOPIC = "/controller/map_zoom_level";

class Mapper {
public:
    Mapper(ros::NodeHandle nh, grid_map::GridMap &global_map, grid_map::GridMap &localmap);
    ~Mapper();

    void setZoom(double zoom);

    grid_map::GridMap getTransformedGridMap();
    cv_bridge::CvImage getTransformedMapImg();
    nav_msgs::OccupancyGrid getTransformedOccupancy();

    void send_map_to_screen();
    void updateTransformedMap();                                                 // Monitor if first map has been received


private:
    // Basic ROS requirements
    ros::NodeHandle nodeHandle_;

    // Getting the map from RTABMAP server
    std::string map_sub_topic_;
    ros::Subscriber map_sub_;
    void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);     // Incoming map from RTABMAP callback
    grid_map::GridMap globalMap_;                                         // This is the total map, centered around the user
    bool receivedMap;  

    // Zoom level
    ros::Subscriber zoom_sub_;
    void incomingZoom(const std_msgs::Float32& msg);

    // Transforming the map to the right zoom & location
    grid_map::GridMap transformedMap_;                                  // The map corrected to the right zoom level & position

    // Resizing the map for display on the dotpad
    grid_map::GridMap scale_transformedMap_to_screen();

    // Sending out the maps
    ros::Publisher output_occupancy_publisher_;
    ros::Publisher output_previewimg_publisher_;
    ros::Publisher output_detailedimg_publisher_;
    tf::TransformListener odom_listener_;

    // ros::Timer mapUpdateTimer_;
    ros::Timer zoomUpdateTimer_;

    void updateZoom();
    float targetZoom_, currentZoom_;
};

