//
// Created by juerd on 16-3-23.
//

#include <tactilemap_with_intent/Mapper.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_eigen.h>

using namespace grid_map;


Mapper::Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map, grid_map::GridMap &localmap) : nodeHandle_(
        nodeHandle) {
    this->nodeHandle_ = nodeHandle;

    /*
     * Subscribers & Publishers
     */
    map_sub_ = nodeHandle_.subscribe("/rtabmap/grid_map", 1, &Mapper::incomingMap, this);
    localmap_occupancy_pub_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("localmap_occupancy", 1);
    localmap_img_pub_ = nodeHandle_.advertise<cv_bridge::CvImage>("localmap_img", 1);

    ROS_INFO("Mapper Started");

    /*
     * Initialize empty GridMaps
     */
    this->globalmap_ = global_map;
    globalmap_.setBasicLayers(globalmap_.getLayers());
    globalmap_.setFrameId("/map");
    this->localmap_ = localmap;
    localmap_.setBasicLayers(localmap_.getLayers());
    localmap.setFrameId("/base_link");

    /*
     * Set timed screen & publish refresh
     */
    double scrnRate = SCRN_DEFAULT_RATE;
    if (!nodeHandle.getParam("/screen_rate_hz", rate)) {
        ROS_INFO("No parameter for screen rate; has node started? -- using default value");
    }
    ROS_INFO("Running at %f hz", rate);
    mapUpdateTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / scrnRate), std::bind(&Mapper::updateSubMap, this));

    /*
     * Set timed zoom level update from parameter server
     */
    double zoomRate = 5;
    updateZoom();
    currentZoom_ = targetZoom_;
    zoomUpdateTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / zoomRate), std::bind(&Mapper::updateZoom, this));
}

Mapper::~Mapper() {}


void Mapper::setZoom(double targetZoom) {
    this->targetZoom_ = targetZoom;
}


cv_bridge::CvImage Mapper::getMapImg(){
    localmap_image_out;
    grid_map::GridMapRosConverter::toCvImage(localmap_, "static", sensor_msgs::image_encodings::MONO8, localmap_image_out);
    return localmap_image_out;
}


void Mapper::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    if (info.width || info.height) {
        GridMapRosConverter::fromOccupancyGrid(*msg, "static", globalmap_);
//        ROS_INFO("Got map %dx%d: Updated to GridMap %fx%f", info.width, info.height, globalmap_.getLength().x(),
//                 globalmap_.getLength().y());
    } else ROS_INFO("Empty map");
}


void Mapper::updateSubMap() {
    if(currentZoomLevel_ != targetZoomLevel_){
        // Temporary, add easing here
        currentZoomLevel_ != targetZoomLevel_;
    }

    tf::StampedTransform inputtransform;
    Eigen::Isometry3d eigentransform;
    grid_map::Position position(0,0);
    grid_map::Length length(currentZoomLevel_, currentZoomLevel_*SCRN_ASPECT_RATIO);
    bool success;

    try {
        odom_listener_.lookupTransform("/camera_link", "/map", ros::Time(0), inputtransform);
        tf::transformTFToEigen(inputtransform,eigentransform);

        //TODO map transformation only in XZ-plane: no vertical movement & limit rotation about the y-axis
        localmap_ = globalmap_.getTransformedMap(eigentransform, "static", "camera_link").getSubmap(position, length, success); //temporary
        publishMap();
        ROS_INFO("Successfully transformed map");
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
};


void Mapper::publishMap() {
//     Publish transformed gridmap
    if (localmap_.getLength().x() && localmap_.getLength().y()) {
        nav_msgs::OccupancyGrid localmap_occupancy_outmsg;
        GridMapRosConverter::toOccupancyGrid(localmap_, "static", 0., 1., localmap_occupancy_outmsg);
        localmap_occupancy_pub_.publish(localmap_occupancy_outmsg);

        cv_bridge::CvImage localmap_image_out;
        grid_map::GridMapRosConverter::toCvImage(localmap_, "static", sensor_msgs::image_encodings::MONO8, localmap_image_out);
        localmap_img_pub_.publish(localmap_image_out);
    } else ROS_INFO("Cannot publish tf map because it's still empty");

}


void updateZoom(){
    if (! nodeHandle_.getParam("zoom_level", targetZoom_)){
        ROS_INFO("Could not find parameter zoom_level; is it up?");
    }
}