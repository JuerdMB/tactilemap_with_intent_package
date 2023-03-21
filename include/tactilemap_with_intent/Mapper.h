//
// Created by juerd on 16-3-23.
//

#ifndef SRC_MAPPER_H
#define SRC_MAPPER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// Layers for map, currently only 1 layer
const std::string STATICLAYER = "staticlayer";
const std::vector<std::string> basicLayers = {STATICLAYER};

#define SCRN_DEFAULT_RATE 5
#define SCRN_ASPECT_RATIO 1.5

class Mapper {
public:
    Mapper(ros::NodeHandle nh, grid_map::GridMap &global_map, grid_map::GridMap &localmap);
    ~Mapper();
    void setZoom(double zoom);

    void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void updateSubMap();
    void publishMap();

    grid_map::GridMap getTransformedGridMap();
    cv_bridge::CvImage getTransformedMapImg();
    nav_msgs::OccupancyGrid getTransformedOccupancy();

private:
    ros::NodeHandle nodeHandle_;
    std::string map_sub_topic_;
    ros::Subscriber map_sub_;
    ros::Publisher transformedmap_occupancy_pub_;
    ros::Publisher transformedmap_img_pub_;
    tf::TransformListener odom_listener_;

    grid_map::GridMap globalmap_;
    grid_map::GridMap transformedmap_;

    ros::Timer mapUpdateTimer_;
    ros::Timer zoomUpdateTimer_;

    void updateZoom();
    double targetZoom_, currentZoom_;
};

#endif //SRC_MAPPER_H
