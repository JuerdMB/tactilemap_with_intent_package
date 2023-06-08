#include "tactilemap_with_intent_package/Mapper.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

Mapper::Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map, grid_map::GridMap &localmap) : nodeHandle_(nodeHandle)
{
    map_sub_ = nodeHandle_.subscribe("/rtabmap/grid_map", 1, &Mapper::incomingMap, this);
    zoom_sub_ = nodeHandle_.subscribe(MAP_ZOOM_TOPIC, 1, &Mapper::incomingZoom, this);

    output_mapdata_publisher_ = nodeHandle_.advertise<std_msgs::UInt8MultiArray>(OUTPUT_DATA_TOPIC, 1);
    output_previewimg_publisher_ = nodeHandle_.advertise<cv_bridge::CvImage>(OUTPUT_PREVIEWIMG_TOPIC, 1);
    output_detailedimg_publisher_ = nodeHandle_.advertise<cv_bridge::CvImage>(OUTPUT_DETAILEDIMG_TOPIC, 1);

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
    nav_msgs::MapMetaData info = msg->info;
    // Check if the map has valid width & height
    if (info.width || info.height)
    {
        grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "static", globalMap_);
        ROS_INFO("Mapper got new map: wxh = %fx%f \t res = %f \t cells = %dx%d \n",
                 globalMap_.getLength()(0), globalMap_.getLength()(1), globalMap_.getResolution(),
                 globalMap_.getSize()(0), globalMap_.getSize()(1));
    }

    if (!receivedMap)
    {
        receivedMap = true;
        updateTransformedMap(); // Create initial transformed map
    }
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
        ROS_DEBUG("Updating transformed map... ");

        // Update and ease the zoom level on the map
        updateZoom();

        // Get the current robot pose
        tf::StampedTransform inputtransform;

        // Set position (0 because translation happens in map transform) and scale for the transformed map.
        grid_map::Position position(0, 0);
        grid_map::Length length(currentZoom_, currentZoom_ / SCRN_ASPECT_RATIO);

        try
        {
            odom_listener_.waitForTransform("/camera_link", "/map", ros::Time(0), ros::Duration(3.0));
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

            bool success;

            // Transform map with isometry
            transformedMap_ = globalMap_.getTransformedMap(isometryTransform, STATICLAYER, "camera_link").getSubmap(position, length, success);

            ROS_DEBUG("Created new, zoomed map of [%dx%d]@res%.2f (%.2fx%2.f[m])",
                      transformedMap_.getSize()(0), transformedMap_.getSize()(1), transformedMap_.getResolution(), transformedMap_.getLength()(0), transformedMap_.getLength()(1));

            // If the transform is successful, publish image of the map to the server
            if (success)
            {
                cv_bridge::CvImage transformedMap_image_out;
                grid_map::GridMapRosConverter::toCvImage(transformedMap_, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                         transformedMap_image_out);
                output_detailedimg_publisher_.publish(transformedMap_image_out);
            }
            else 
                ROS_ERROR("Map transform failed");
        }

        catch (tf::TransformException ex)
        {
            ROS_ERROR("UpdateTransformedMap: %s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    else
    {
        // No map received yet
        ROS_INFO("Cannot update transformed map: No map received yet");
    }
};

// SENDING OUT THE MAPS

void Mapper::publishMap()
{ // Send map to the screen for display

    if (receivedMap)
    {
        // Change resolution of the map to be total of 60x40px
        grid_map::GridMap scaledMap;
        MapDataConverter::mapToScreenResolution(transformedMap_, scaledMap);

        // Convert the output map to a CV Image and publish to 60x40 preview topic
        cv_bridge::CvImage transformedMap_image_out;
        grid_map::GridMapRosConverter::toCvImage(scaledMap, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                 transformedMap_image_out);
        output_previewimg_publisher_.publish(transformedMap_image_out);

        // Convert the map to an array of bytes (uint8) and publish to message for dotpad receiver
        std::vector<uint8_t> data = MapDataConverter::mapToDataArray(scaledMap, STATICLAYER);

        if (data.size() != 0)
        {
            std_msgs::UInt8MultiArray screendata_msg; // Construct the message
            screendata_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            screendata_msg.layout.dim[0].size = data.size();
            screendata_msg.layout.dim[0].stride = 1;
            screendata_msg.layout.dim[0].label = "data";
            screendata_msg.data.insert(screendata_msg.data.end(), data.begin(), data.end());

            output_mapdata_publisher_.publish(screendata_msg);
            ROS_INFO("Published map data as a vector of size %d", screendata_msg.data.size());
        }
        else
            ROS_ERROR("Cannot publish empty vector");
    }
}

