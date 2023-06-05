#include "tactilemap_with_intent_package/Mapper.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <grid_map_cv/grid_map_cv.hpp>

using namespace grid_map;

Mapper::Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map, grid_map::GridMap &localmap) : nodeHandle_(nodeHandle)
{
    map_sub_ = nodeHandle_.subscribe("/rtabmap/grid_map", 1, &Mapper::incomingMap, this);
    zoom_sub_ = nodeHandle_.subscribe(MAP_ZOOM_TOPIC, 1, &Mapper::incomingZoom, this);

    output_occupancy_publisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("outputmap_occupancy", 1);
    output_previewimg_publisher_ = nodeHandle_.advertise<cv_bridge::CvImage>("outputmap_preview_img", 1);
    output_detailedimg_publisher_ = nodeHandle_.advertise<cv_bridge::CvImage>("outputmap_detailed_img", 1);

    receivedMap = false;

    // Initialize empty GridMaps
    this->globalMap_ = global_map;
    globalMap_.setBasicLayers(globalMap_.getLayers());
    globalMap_.setFrameId("/map");
    this->transformedMap_ = localmap;
    transformedMap_.setBasicLayers(transformedMap_.getLayers());
    localmap.setFrameId("/camera_link");

    // Rate for zoom level update from parameter server
    updateZoom();
    currentZoom_ = targetZoom_;
    zoomUpdateTimer_ = nodeHandle_.createTimer(ros::Duration(1.0 / ZOOM_DEFAULT_RATE), std::bind(&Mapper::updateZoom, this));
}

Mapper::~Mapper() {}

// RECEIVING MAPS FROM MAP SERVER

void Mapper::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if (!receivedMap)
    {
        receivedMap = true;
    }

    nav_msgs::MapMetaData info = msg->info;
    // Check if the map has valid width & height
    if (info.width || info.height)
    {
        GridMapRosConverter::fromOccupancyGrid(*msg, "static", globalMap_);
        ROS_INFO("Got map from map server: %dx%d", info.width, info.height);

        // updateTransformedMap();
    }
    else
        ROS_INFO("Got an empty map from topic %s", this->map_sub_topic_);
}

// ZOOM LEVEL

void Mapper::setZoom(double targetZoom)
{
    this->targetZoom_ = targetZoom;
}

void Mapper::incomingZoom(const std_msgs::Float32 &incomingZoom)
{
    this->targetZoom_ = incomingZoom.data;
    ROS_INFO("received map zoom: %f", incomingZoom.data);
}

void Mapper::updateZoom()
{
    // Try to update zoomlevel
    if (!nodeHandle_.getParam("zoom_level", targetZoom_))
    {
        ROS_INFO("Could not find parameter zoom_level; is it up?");
        targetZoom_ = ZOOM_DEFAULT;
        currentZoom_ = ZOOM_DEFAULT;
    }

    // Update the zoom level of the map
    if (abs(currentZoom_ - targetZoom_) > ZOOMERROR_THRESHOLD)
    {
        // Temporary, add easing here
        currentZoom_ = targetZoom_;
    }
}

// TRANSFORMED MAPS

void Mapper::updateTransformedMap()
{
    if (receivedMap)
    {

        // Update and ease the zoom level on the map
        updateZoom();

        // Get the current robot pose
        tf::StampedTransform inputtransform;

        // Set position (0 because translation happens in map transform) and scale for the transformed map.
        grid_map::Position position(0, 0);
        grid_map::Length length(currentZoom_, currentZoom_ / SCRN_ASPECT_RATIO);
        bool success;

        try
        {
            odom_listener_.lookupTransform("/camera_link", "/map", ros::Time(0), inputtransform);

            // Transform inputtransform rotation to euler angles
            tfScalar roll, pitch, yaw;
            inputtransform.getBasis().getRPY(roll, pitch, yaw);
            roll = 0.;
            pitch = 0.;

            // Transform euler angles back into quaternion and create xz-transform
            tf::Quaternion xz_quat(roll, pitch, yaw);
            tf::Transform xz_transform(xz_quat, inputtransform.getOrigin());

            // Turn tf transform into isometry3D for map translation
            Eigen::Isometry3d isometryTransform;
            tf::transformTFToEigen(xz_transform, isometryTransform);

            // Transform map with isometry
            transformedMap_ = globalMap_.getTransformedMap(isometryTransform, STATICLAYER, "camera_link")
                                  .getSubmap(position, length, success);


            // If the transform is successful, publish image of the map to the server
            if (success)
            {
                cv_bridge::CvImage transformedMap_image_out;
                grid_map::GridMapRosConverter::toCvImage(transformedMap_, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                         transformedMap_image_out);
                output_detailedimg_publisher_.publish(transformedMap_image_out);
            }

            else
                ROS_INFO("Could not transform");
        }

        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    else
    {

        ROS_WARN("Requested to transform map but no map received yet!");
    }
};

// SENDING OUT THE MAPS

void Mapper::send_map_to_screen()
{ // Send map to the screen for display

    if (receivedMap)
    {

        // Calculate the resolution for total size of 60x40 px
        double res = transformedMap_.getLength()(0) / 60.;

        // Change resolution of the map to be total of 60x40px
        GridMap scaledmap;
        grid_map::GridMapCvProcessing::changeResolution(transformedMap_, scaledmap, res);

        ROS_INFO("Sending map to screen %d x %d", scaledmap.getSize()(0), scaledmap.getSize()(1));

        // Convert the map into an occupancy grid message and publish
        nav_msgs::OccupancyGrid occupancy_msg;
        GridMapRosConverter::toOccupancyGrid(scaledmap, STATICLAYER, 0., 1., occupancy_msg);
        output_occupancy_publisher_.publish(occupancy_msg);

        // Convert the output map to a CV Image and publish to 60x40 preview topic
        cv_bridge::CvImage transformedMap_image_out;
        grid_map::GridMapRosConverter::toCvImage(scaledmap, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                 transformedMap_image_out);
        output_previewimg_publisher_.publish(transformedMap_image_out);
    }

    else
        ROS_ERROR("Have not received map yet");
}
