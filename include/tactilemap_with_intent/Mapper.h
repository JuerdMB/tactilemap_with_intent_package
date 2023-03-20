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

const std::vector<std::string> basicLayers = {"static", "dynamic"};

#define SCRN_DEFAULT_RATE 5
#define SCRN_ASPECT_RATIO 1.5

class Mapper {
public:
    Mapper(ros::NodeHandle nh, grid_map::GridMap &global_map, grid_map::GridMap &localmap);
    ~Mapper();
    void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void setZoom(double zoom);
    void updateSubMap();
    void publishMap();
    cv_bridge::CvImage getMapImg();

private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber map_sub_;
    ros::Publisher localmap_occupancy_pub_;
    ros::Publisher localmap_img_pub_;
    tf::TransformListener odom_listener_;

    grid_map::GridMap globalmap_;
    grid_map::GridMap localmap_;

    ros::Timer mapUpdateTimer_;
    ros::Timer zoomUpdateTimer_;

    double targetZoom_, currentZoom_;
};

#endif //SRC_MAPPER_H
