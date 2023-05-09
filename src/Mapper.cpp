//
// Created by juerd on 16-3-23.
//

#include "tactilemap_with_intent_package/Mapper.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

using namespace grid_map;

/*
 * Constructor
 */
Mapper::Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map, grid_map::GridMap &localmap) :
        nodeHandle_(nodeHandle) {

    ROS_INFO("Mapper Started");

    // Subscribers & Publishers
    this->nodeHandle_ = nodeHandle;
    map_sub_ = nodeHandle_.subscribe("/rtabmap/grid_map", 1, &Mapper::incomingMap, this);
    transformedmap_occupancy_pub_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("transformedmap_occupancy", 1);
    transformedmap_img_pub_ = nodeHandle_.advertise<cv_bridge::CvImage>("transformedmap_img", 1);

    // Initialize empty GridMaps
    this->globalmap_ = global_map;
    globalmap_.setBasicLayers(globalmap_.getLayers());
    globalmap_.setFrameId("/map");
    this->transformedmap_ = localmap;
    transformedmap_.setBasicLayers(transformedmap_.getLayers());
    localmap.setFrameId("/base_link");

    // Rate of map updates and publishes
    double scrnRate = SCRN_DEFAULT_RATE;
    if (nodeHandle.getParam("/screen_rate_hz", scrnRate)) {
        ROS_INFO("Running at %f hz", scrnRate);
    } else {
        ROS_INFO("No parameter for screen rate; has node started? -- using default value");
    }
    mapUpdateTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / scrnRate), std::bind(&Mapper::updateSubMap, this));

    // Rate for zoom level update from parameter server
    double zoomRate = 5.;
    updateZoom();
    currentZoom_ = targetZoom_;
    zoomUpdateTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / zoomRate), std::bind(&Mapper::updateZoom, this));
}

/*
 * Default destructor
 */
Mapper::~Mapper() {}

/*
 * Zoom level setter for the TMWI
 */
void Mapper::setZoom(double targetZoom) {
    this->targetZoom_ = targetZoom;
}

/*
 * Get zoom from parameter server.
 */
void Mapper::updateZoom() {
    if (!nodeHandle_.getParam("zoom_level", targetZoom_)) {
        ROS_INFO("Could not find parameter zoom_level; is it up?");
    }
}

/*
 * Callback function for when new map is obtained
 */
void Mapper::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    nav_msgs::MapMetaData info = msg->info;
    // Check if the map has valid width & height
    if (info.width || info.height) {
        GridMapRosConverter::fromOccupancyGrid(*msg, "static", globalmap_);
        ROS_INFO("Got map from map server: %dx%d", info.width, info.height);
    } else ROS_INFO("Got an empty map from topic %s", this->map_sub_topic_);
}

/*
 * This method runs everytime the screen is updated
 */
void Mapper::updateSubMap() {

    // Update the zoom level of the map
    if (currentZoom_ != targetZoom_) {
        // Temporary, add easing here
        currentZoom_ = targetZoom_;
    }

    // Get the current robot pose
    tf::StampedTransform inputtransform;

    // Set position (0 because translation happens in map transform) and scale for the transformed map.
    grid_map::Position position(0, 0);
    grid_map::Length length(currentZoom_, currentZoom_*SCRN_ASPECT_RATIO);
    bool success;

    try {
        odom_listener_.lookupTransform("/camera_link", "/map", ros::Time(0), inputtransform);

        // Transform inputtransform rotation to euler angles
        tfScalar roll, pitch, yaw;
        inputtransform.getBasis().getRPY(roll, pitch, yaw);
        roll = 0; pitch = 0;

        // Transform euler angles back into quaternion and create xz-transform
        tf::Quaternion xz_quat(yaw, pitch, roll);
        tf::Transform xz_transform(xz_quat, inputtransform.getOrigin());

        // Turn tf transform into isometry3D for map translation
        Eigen::Isometry3d isometryTransform;
        tf::transformTFToEigen(xz_transform, isometryTransform);

        //TODO Low pass filter op de transform zetten
        //TODO eerst submap, dan transform, dan weer submap

        // Transform map with isometry
        transformedmap_ = globalmap_.getTransformedMap(isometryTransform, STATICLAYER, "camera_link")
                .getSubmap(position, length, success);

        // Publish map to map topic
        publishMap();
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
};

/*
 * Publish the transformed zoomed submap
 */
void Mapper::publishMap() {
//     Publish transformed gridmap
    if (transformedmap_.getLength().x() && transformedmap_.getLength().y()) {
        nav_msgs::OccupancyGrid transformedmap_occupancy_outmsg;
        GridMapRosConverter::toOccupancyGrid(transformedmap_, basicLayers, 0., 1., transformedmap_occupancy_outmsg);
        transformedmap_occupancy_pub_.publish(transformedmap_occupancy_outmsg);

        cv_bridge::CvImage transformedmap_image_out;
        grid_map::GridMapRosConverter::toCvImage(transformedmap_, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                 transformedmap_image_out);
        transformedmap_img_pub_.publish(transformedmap_image_out);
    } else ROS_INFO("Cannot publish tf map because it's still empty");

}