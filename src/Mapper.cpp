#include "tactilemap_with_intent_package/Mapper.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

using namespace grid_map;

Mapper::Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map, grid_map::GridMap &localmap) : nodeHandle_(nodeHandle)
{
    // Initial ROS stuff
    this->nodeHandle_ = nodeHandle;

    map_sub_ = nodeHandle_.subscribe("/rtabmap/grid_map", 1, &Mapper::incomingMap, this);
    zoom_sub_ = nodeHandle_.subscribe(MAP_ZOOM_TOPIC, 1, &Mapper::incomingZoom, this);
    transformedMap_occupancy_pub_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("transformedMap_occupancy", 1);
    transformedMap_img_pub_ = nodeHandle_.advertise<cv_bridge::CvImage>("transformedMap_img", 1);

    receivedMap = false;

    // Initialize empty GridMaps
    this->fullMap_ = global_map;
    fullMap_.setBasicLayers(fullMap_.getLayers());
    fullMap_.setFrameId("/map");
    this->transformedMap_ = localmap;
    transformedMap_.setBasicLayers(transformedMap_.getLayers());
    localmap.setFrameId("/base_link");

    // Rate of map updates and publishes
    double scrnRate = SCRN_DEFAULT_RATE;
    if (nodeHandle.getParam("/screen_rate_hz", scrnRate))
    {
        ROS_INFO("Running at %f hz", scrnRate);
    }
    else
    {
        ROS_INFO("No parameter for screen rate; has node started? -- using default value");
    }

    // Rate for zoom level update from parameter server
    double zoomRate = 5.;
    updateZoom();
    currentZoom_ = targetZoom_;

}

Mapper::~Mapper() {}

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
        GridMapRosConverter::fromOccupancyGrid(*msg, "static", fullMap_);
        ROS_INFO("Got map from map server: %dx%d", info.width, info.height);
        
        //transformFullMapToMe();
    }
    else
        ROS_INFO("Got an empty map from topic %s", this->map_sub_topic_);
}


void Mapper::setZoom(double targetZoom)
{
    this->targetZoom_ = targetZoom;
}

/*
 * CALLBACKS
*/

void Mapper::incomingZoom(const std_msgs::Float32& incomingZoom){
    this->targetZoom_ = incomingZoom.data;
    ROS_INFO("received map zoom: %f", incomingZoom.data);
}

/*
 * Methods for updating the map
*/

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

void Mapper::transformFullMapToMe()
{
    if (receivedMap)
    {

        // Update and ease the zoom level on the map
        updateZoom();

        // Get the current robot pose
        tf::StampedTransform inputtransform;

        // Set position (0 because translation happens in map transform) and scale for the transformed map.
        grid_map::Position position(0, 0);
        grid_map::Length length(currentZoom_, currentZoom_ * SCRN_ASPECT_RATIO);
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
            transformedMap_ = fullMap_.getTransformedMap(isometryTransform, STATICLAYER, "camera_link")
                                  .getSubmap(position, length, success);

            // Only publish if transform was succesful
            if (success)
            {
                const double x = transformedMap_.getLength().x();
                const double y = transformedMap_.getLength().y();

                if (x && y)
                {
                    ROS_INFO("Transformed Global map: Zoomlvl = %f \t Size = [%f, %f]", currentZoom_, x, y);

                    // Publish map to map & map image
                    publish_map_image();
                }
                else
                    ROS_INFO("Maps contain no geometry");
            }
            else
                ROS_INFO("Could not transform");
        }

        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    } // End if(receivedMap)
    else ROS_WARN("No map received!");
};


// Update the display of current TF map for display in RVIZ
void Mapper::publish_map_image()
{
    if (transformedMap_.getLength().x() && transformedMap_.getLength().y())
    {
        // Transform a CVImage of the map to the server
        cv_bridge::CvImage transformedMap_image_out;
        grid_map::GridMapRosConverter::toCvImage(transformedMap_, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                 transformedMap_image_out);
        
        transformedMap_img_pub_.publish(transformedMap_image_out);
    }
    else
    {
        ROS_INFO("Attempted to publish image of TF map but it's still empty");
        ros::Duration(1.0).sleep();
    }
}





// Send map to the screen for display

void Mapper::scale_transformedMap_to_screen(nav_msgs::OccupancyGrid * scaledMap){

    // Take transformedMap, resize it and save to output_gridmap

}

void Mapper::send_map_to_screen(){

    transformFullMapToMe();

    scale_transformedMap_to_screen();

    if (output_gridmap.getLength().x() && output_gridmap.getLength().y())
    {
        ROS_INFO("Sending map to screen %f x %f", transformedMap_.getLength().x(), transformedMap_.getLength().y());
        nav_msgs::OccupancyGrid transformedMap_occupancy_outmsg;
        GridMapRosConverter::toOccupancyGrid(transformedMap_, basicLayers[0], 0., 1., transformedMap_occupancy_outmsg);
        transformedMap_occupancy_pub_.publish(transformedMap_occupancy_outmsg);
    }
    else
    {
        ROS_INFO("Cannot publish tf map because it's still empty");
        ros::Duration(1.0).sleep();
    }
}