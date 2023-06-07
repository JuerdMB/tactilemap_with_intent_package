#include "tactilemap_with_intent_package/Mapper.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <grid_map_cv/grid_map_cv.hpp>

const std::string SCREEN_DATA_TOPIC = "/topic";

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
    if (!receivedMap)
        receivedMap = true;

    nav_msgs::MapMetaData info = msg->info;
    // Check if the map has valid width & height
    if (info.width || info.height)
    {
        GridMapRosConverter::fromOccupancyGrid(*msg, "static", globalMap_);
        ROS_INFO("Mapper got new map: wxh = %fx%f \t res = %f \t cells = %dx%d \n",
                 globalMap_.getLength()(0), globalMap_.getLength()(1), globalMap_.getResolution(),
                 globalMap_.getSize()(0), globalMap_.getSize()(1));
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

        // Update and ease the zoom level on the map
        updateZoom();

        // Get the current robot pose
        tf::StampedTransform inputtransform;

        // Set position (0 because translation happens in map transform) and scale for the transformed map.
        grid_map::Position position(0, 0);
        grid_map::Length length(currentZoom_, currentZoom_ / SCRN_ASPECT_RATIO);

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

            bool success;

            // Transform map with isometry
            transformedMap_ = globalMap_.getTransformedMap(isometryTransform, STATICLAYER, "camera_link");
            zoomedMap_ = transformedMap_.getSubmap(position, length, success);

            ROS_INFO( "Transformed map to %fx%f [m], with a resolution of %f \t %dx%d cells", 
                                                    zoomedMap_.getLength()(0), zoomedMap_.getLength()(1), zoomedMap_.getResolution(),
                                                    zoomedMap_.getSize()(0), zoomedMap_.getSize()(1) );

            // If the transform is successful, publish image of the map to the server
            if (success)
            {
                cv_bridge::CvImage transformedMap_image_out;
                grid_map::GridMapRosConverter::toCvImage(transformedMap_, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                         transformedMap_image_out);
                output_detailedimg_publisher_.publish(transformedMap_image_out);
            }
        }

        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    else
    {
        // No map received yet
    }
};

// SENDING OUT THE MAPS

void Mapper::publishMap()
{ // Send map to the screen for display

    if (receivedMap)
    {
        // Change resolution of the map to be total of 60x40px
        grid_map::GridMap scaledMap;
        mapToScreenResolution(transformedMap_, scaledMap);

        ROS_DEBUG("Sending map to screen %d x %d", scaledMap.getSize()(0), scaledMap.getSize()(1));

        // Convert the output map to a CV Image and publish to 60x40 preview topic
        cv_bridge::CvImage transformedMap_image_out;
        grid_map::GridMapRosConverter::toCvImage(scaledMap, STATICLAYER, sensor_msgs::image_encodings::MONO8,
                                                 transformedMap_image_out);
        output_previewimg_publisher_.publish(transformedMap_image_out);


        // Convert the map to an array of bytes (uint8) and publish on the 
        std_msgs::UInt8MultiArray screendata_msg;
        std_msgs::MultiArrayLayout screendata_msg_layout;
        screendata_msg.data = mapToDataArray(scaledMap);
        output_mapdata_publisher_.publish(screendata_msg);
    }
}

void Mapper::mapToScreenResolution(grid_map::GridMap &inputMap, grid_map::GridMap &outputMap){
    // Calculate the resolution for total size of 60x40 px
    double res = transformedMap_.getLength()(0) / 60.;using namespace grid_map;

    // Implementation for resolution change
    grid_map::GridMapCvProcessing::changeResolution(inputMap, outputMap, res);
}

std::vector<uint8_t> Mapper::mapToDataArray(grid_map::GridMap &inputMap){

    int mapWidth = inputMap.getSize()(0);
    int mapHeight = inputMap.getSize()(1);

    if(inputMap.getSize()(0) != 60 || inputMap.getSize()(1) != 40) {
        ROS_ERROR("mapToDataArray received map of incorrect dimensions %dx%d", mapWidth, mapHeight);
        return {};
    }

    std::vector<uint8_t> tempVector;

    // Loop through 'braille cells' (collection of 8 map cells per braille cell), first by row, then by column

    int curCell = 0;    // This variable stores the current byte in tempVector that we wil be writing bits to

    for(int braille_cell_row=0; braille_cell_row<10; braille_cell_row++) { // Rows

        for (int braille_cell_column=0; braille_cell_column<30; braille_cell_column++){
            
            // Braille cells are 4 rows high, 2 columns wide
            int map_row_start = braille_cell_row * 4;
            int map_column_start = braille_cell_column * 2;

            // We have the start coordinates of this braille cell, now loop through each of its cells

            // Loop through each of the dots of this cell
            for(int x = map_column_start; x < map_column_start+2; x++){ //Loop through first column, then second

                for(int y=map_row_start; y<map_row_start+4; y++){ // Loop through each of the for vertical rows per column

                    grid_map::Index curIndex(x, y);

                    uint8_t curPixel = inputMap.at(STATICLAYER, curIndex);

                    if (curPixel > 1) {
                        ROS_ERROR("Input value cannot exceed 1");
                    }

                    if(curPixel < 0){
                        // Got an uncertainty cell
                    }

                    uint8_t curBitLoc = (x-map_column_start)*4 + (y-map_row_start);  // If in second row, add 4 to y to obtain location

                    tempVector[curCell] |= curPixel << (7-curBitLoc);
//                    std::printf("(%d)%d ", curBitLoc,curPixel);
                }
            }

            curCell++;
        }
    }

    return tempVector;
}