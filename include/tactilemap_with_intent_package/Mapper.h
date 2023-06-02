//
// Created by juerd on 16-3-23.
//

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

const std::string MAP_ZOOM_TOPIC = "/controller/map_zoom_level";

class Mapper {
public:
    Mapper(ros::NodeHandle nh, grid_map::GridMap &global_map, grid_map::GridMap &localmap);
    ~Mapper();
    void setZoom(double zoom);

    

    grid_map::GridMap getTransformedGridMap();
    cv_bridge::CvImage getTransformedMapImg();
    nav_msgs::OccupancyGrid getTransformedOccupancy();

private:
    ros::NodeHandle nodeHandle_;
    std::string map_sub_topic_;
    ros::Subscriber map_sub_;
    ros::Subscriber map_zoom_sub_;
    ros::Publisher transformedmap_occupancy_pub_;
    ros::Publisher transformedmap_img_pub_;
    tf::TransformListener odom_listener_;

    bool receivedMap; // Monitor if first map has been received
    grid_map::GridMap globalmap_;
    grid_map::GridMap transformedmap_;

    void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg); // Map callback
    void incomingZoom(const std_msgs::Float32& msg);
    void updateTransformedMap();
    void publishMap();

    ros::Timer mapUpdateTimer_;
    ros::Timer zoomUpdateTimer_;

    void updateZoom();
    float targetZoom_, currentZoom_;
};

