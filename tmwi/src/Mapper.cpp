#include "tmwi/Mapper.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <grid_map_msgs/GridMap.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

const float PI = 3.1415;

grid_map::GridMapRosConverter converter;

Mapper::Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map) : nodeHandle_(nodeHandle)
{
    sub_rtabmap_ = nodeHandle_.subscribe(TOPIC_INPUT_MAP, 1, &Mapper::incomingMap, this);

    pub_bytes_dotpad = nodeHandle_.advertise<std_msgs::UInt8MultiArray>(TOPIC_DOTPAD_DATA, 1);
    pub_image_screensized = nodeHandle_.advertise<cv_bridge::CvImage>(TOPIC_IMAGE_SCREENSIZED, 1);
    pub_image_highres = nodeHandle_.advertise<cv_bridge::CvImage>(TOPIC_IMAGE_DETAILED, 1);

    // Initialize empty global GridMap
    this->globalMap_ = global_map;
    globalMap_.setBasicLayers(basicLayers);
    globalMap_.setFrameId("/map");

    receivedMap = false;

    if (!nodeHandle_.getParam(PARAM_ZOOM_LEVEL, zoom_level))
        zoom_level = DEFAULT_ZOOM;


    ROS_INFO("Set up Mapper");
}

Mapper::~Mapper() {}

// RECEIVING MAPS FROM MAP SERVER

void Mapper::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_INFO("Received Map from RTABMAP");

    nav_msgs::MapMetaData info = msg->info;
    // Check if the map has valid width & height
    if (info.width || info.height)
    {
        grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "static", globalMap_);
        ROS_INFO("Map received: length.x=%f length.y=%f\t resolution = %f \t cells.x=%d cells.y=%d",
                 globalMap_.getLength().x(), globalMap_.getLength().y(), globalMap_.getResolution(),
                 globalMap_.getSize()(0), globalMap_.getSize()(1));
    }

    if (!receivedMap)
        receivedMap = true;
}

// TRANSFORMED MAPS

TRANSFORM_ERROR Mapper::getTransformedZoomedMap(grid_map::GridMap &transformedZoomedMap)
{
    if (receivedMap)
    {
        try
        {
            // Get the current zoom level
            if (!nodeHandle_.getParam(PARAM_ZOOM_LEVEL, zoom_level))
            {
                return ZOOM_LOOKUP_ERROR;
            }

            // TODO get the zoom from parameter server
            float length_x = zoom_level;                       // X-richting, hoogte van de map    = zoom_level
            float length_y = zoom_level * SCREEN_ASPECT_RATIO; // Y-richting, breedte van de map
            grid_map::Length length(length_x, length_y);

            // Try to get the current robot pose
            tf::StampedTransform robot_pose;
            odom_listener_.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
            odom_listener_.lookupTransform("/base_link", "/map", ros::Time(0), robot_pose);

            // Transform robot_pose rotation to euler angles
            tfScalar roll, pitch, yaw;
            robot_pose.getBasis().getRPY(roll, pitch, yaw);
            roll = 0.;
            pitch = 0.;

            // Transform euler angles back into quaternion and create xz-transform
            tf::Quaternion xz_quat(roll, pitch, yaw);
            tf::Transform xz_transform(xz_quat, robot_pose.getOrigin());

            // Turn tf transform into isometry3D for map translation
            Eigen::Isometry3d isometryTransform;
            tf::transformTFToEigen(xz_transform, isometryTransform);

            bool success;

            // Transform map with isometry
            transformedZoomedMap = globalMap_.getTransformedMap(isometryTransform, STATICLAYER, "base_link").getSubmap(grid_map::Position(0, 0), length, success);

            // Return
            if (success)
                return TRANSFORM_ERROR_NONE;
            else
                return SUBMAP_ERROR;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("UpdatetransformedZoomedMap: %s", ex.what());
            return ODOM_LOOKUP_ERROR;
        }
    }
    else // No map received yet
        return NO_MAP_RECEIVED;
};

void Mapper::publishTransformedZoomedMap()
{ // Send map to the screen for display

    if (receivedMap)
    {
        // Change resolution of the map to be total of 60x40px
        grid_map::GridMap transformedZoomedMap;
        transformedZoomedMap.setBasicLayers(globalMap_.getBasicLayers());
        TRANSFORM_ERROR result = getTransformedZoomedMap(transformedZoomedMap); // TODO deze functie grondig checken of transforms kloppen

        if (result != TRANSFORM_ERROR_NONE)
        {
            // Something went wrong
            switch (result)
            {
            case NO_MAP_RECEIVED:
                ROS_INFO("Cannot update transformed map: No map received yet");
                break;
            case SUBMAP_ERROR:
                ROS_ERROR("SubMap error");
                break;
            case ZOOM_LOOKUP_ERROR:
                ROS_ERROR("Zoom parameter lookup failed");
                break;
            case ODOM_LOOKUP_ERROR:
                ROS_ERROR("Odometry lookup failed");
                break;
            default:
                ROS_ERROR("Unknown error!");
                break;
            }
            return;
        }

        ROS_INFO("Transformed map from %.2fx%.2f[m](%2dx%2d cells) to:\twidth: %.2f[m] height: %.2f[m] cells: x=%2d y=%2d",
                 globalMap_.getLength()(1), globalMap_.getLength()(0), globalMap_.getSize()(1), globalMap_.getSize()(0),
                 transformedZoomedMap.getLength()(1), transformedZoomedMap.getLength()(0), transformedZoomedMap.getSize()(1), transformedZoomedMap.getSize()(0));

        // SCALING THE MAP TO CORRECT RESOLUTION FOR DISPLAY

        grid_map::GridMap screensizedMap;
        screensizedMap.setBasicLayers(transformedZoomedMap.getBasicLayers());
        Mapper::mapToScreenResolution(transformedZoomedMap, screensizedMap); // TODO deze vervangen met een custom functie die strenger is

        ROS_INFO("Rescaled to resolution %.3f [cells/m]: width=%2d height=%2d ",
                 screensizedMap.getResolution(), screensizedMap.getSize()(1), screensizedMap.getSize()(0));

        // PUBLISH MAP PREVIEWS
        cv_bridge::CvImage map_image_out;

        // HIGH RES TRANSFORMED
        map_image_out = get_image_from_map(transformedZoomedMap, STATICLAYER);
        pub_image_highres.publish(map_image_out);

        // SCREEN RESOLUTION PREVIEW | IMAGE & GRIDMAP
        map_image_out = get_image_from_map(screensizedMap, STATICLAYER);
        pub_image_screensized.publish(map_image_out);

        // // Convert the map to an array of bytes (uint8) and publish to message for dotpad receiver
        std::vector<uint8_t> data = Mapper::getDataArray(screensizedMap, STATICLAYER);
        if (data.size() != 0)
        {
            std_msgs::UInt8MultiArray screendata_msg; // Construct the message
            screendata_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            screendata_msg.layout.dim[0].size = data.size();
            screendata_msg.layout.dim[0].stride = 1;
            screendata_msg.layout.dim[0].label = "data";
            screendata_msg.data.insert(screendata_msg.data.end(), data.begin(), data.end());

            pub_bytes_dotpad.publish(screendata_msg);
            // ROS_INFO("Published map data as a vector of size %d", screendata_msg.data.size());
        }
        else
            ROS_ERROR("Cannot publish empty vector");
    }
}

// STATIC HELPER FUNCTIONS

void Mapper::mapToScreenResolution(grid_map::GridMap &inputMap, grid_map::GridMap &outputMap)
{
    // Calculate the resolution for total size of 60x40 px
    double res = inputMap.getLength().y() / SCREEN_WIDTH; // Breedterichting (Y-richting in een GridMap) moet 60 px worden

    // Implementation for resolution change
    grid_map::GridMapCvProcessing::changeResolution(inputMap, outputMap, res);
}

cv_bridge::CvImage Mapper::get_image_from_map(grid_map::GridMap &input_map, const std::string &layer)
{
    cv_bridge::CvImage outputImage;
    grid_map::GridMapRosConverter::toCvImage(input_map, layer, sensor_msgs::image_encodings::RGB8, 0., 1., outputImage);

    // set current location (center) as red dot
    int numChannels = outputImage.image.channels();
    int width = outputImage.image.cols;
    int height = outputImage.image.rows;
    int centerLoc[2] = {width / 2, height / 2};

    // Loop through all pixels and change contrast
    // for (int y=0; y<height; y++)
    // {
    //     for (int x=0; x<width; x++)
    //     {
    //         ROS_INFO("%d, %d", x, y);
    //         cv::Vec3b & pixel = outputImage.image.at<cv::Vec3b>(cv::Point(y, x));
    //         for (int c=0; c<numChannels; c++)
    //         {
    //             float weightedavg = (float)pixel[c]*contrast + 128.*(1.0-contrast);
    //             pixel[c] = (int)weightedavg;
    //         }
    //     }
    // }

    // Paint selected location red
    std::vector<cv::Point> pointsToPaint = {
        cv::Point(centerLoc[0], centerLoc[1] - 1), // 1 above center location
        cv::Point(centerLoc[0], centerLoc[1]),     // center location
        cv::Point(centerLoc[0], centerLoc[1] + 1)  // 1 below center location
    };
    for (cv::Point point : pointsToPaint)
    {
        cv::Vec3b &pixel = outputImage.image.at<cv::Vec3b>(point);
        pixel[0] = 255.; // Blue channel
        pixel[1] = 0.;   // Green channel
        pixel[2] = 0.;   // Red channel
    }

    // ROS_INFO("Editing CvImage: size[wxh]= %3dx%3d, centerpixel=[%3d,%3d], channels=%d, largest value = %f", width, height, centerLoc[0], centerLoc[1], numChannels, largestVal);

    return outputImage;
}

std::vector<uint8_t> Mapper::getDataArray(grid_map::GridMap &inputMap, const std::string &layer)
{
    int mapWidth = inputMap.getSize()(1);
    int mapHeight = inputMap.getSize()(0);

    if (mapWidth != 60)
    {
        ROS_ERROR("mapToDataArray: received map of incorrect dimensions %dx%d, returning empty vector", mapWidth, mapHeight);
        return {};
    }

    if (mapHeight > 40)
        ROS_INFO("mapToDataArray: received map of correct width but incorrect height %d, cutting of last %d rows", mapHeight, mapHeight - 40);

    // IF the amount of vertical rows in the map < 40, we cannot fill the whole screen;
    // We can only loop through the

    int lastRow = 40;

    if (mapHeight < 40)
    {
        ROS_INFO("mapToDataArray: input map insufficient length %d", mapHeight);
        lastRow = mapHeight;
    }

    std::vector<uint8_t> output_data_vector(300);                       // Empty vector of full screen size
    std::fill(output_data_vector.begin(), output_data_vector.end(), 0); // 0-initialize all values

    // Loop through 'braille cells' (collection of 8 map cells per braille cell), first by row, then by column
    // char debugstring[100] = "";

    for (int braille_cell = 0; braille_cell < 300; braille_cell++)
    {
        // Determine the start x & y values of this braille cell in the map
        int col_start = (braille_cell * 2) % 60;
        int row_start = ((braille_cell - braille_cell % 30) / 30) * 4;

        // Print for debug
        // ROS_INFO("Braille Cell %3d: Col_start=%3d, Row_start=%3d", braille_cell, col_start, row_start);
        // sprintf(debugstring, "%3d\tx0=%2d y0=%2d\tbytes= ", braille_cell, col_start, row_start);

        // Loop through each of the dots of this cell and write them to a bit in the output_data_vector
        for (int x = col_start; x < col_start + 2; x++)
        { // Loop through first column, then second

            for (int y = row_start; y < row_start + 4; y++)
            { // Per column, loop through each of the for vertical rows
                // Determine current bit position in byte, and push the bit into the byte
                uint8_t curPixel;
                uint8_t curBitLoc = (x - col_start) * 4 + (y - row_start); // If in second row, add 4 to y to obtain location

                if (y < lastRow)
                {
                    grid_map::Index curIndex(y, x);
                    curPixel = inputMap.at(layer, curIndex);

                    if (curPixel > 1)
                    {
                        // ROS_INFO("Cell at %2d,%2d > 1, constrained to 1", x, y);
                        curPixel = 1;
                    }

                    if (curPixel < 0)
                    {
                        // ROS_INFO("Cell at %2d,%2d < 0, constrained to 0", x, y);
                        // Got an uncertainty cell
                        curPixel = 0;
                    }
                }

                else
                {
                    // Default 0 value if we dont have enough vertical data (less than 40 rows)
                    curPixel = 0;
                    // ROS_WARN("Insufficient rows in inputmap: %3d; Setting value of %2dth bit of %3dth byte in output array to 0", lastRow, curBitLoc, braille_cell);
                }
                // sprintf(debugstring, "%s %d ", debugstring, (unsigned int)curPixel);
                output_data_vector[braille_cell] |= curPixel << (curBitLoc);
            }
        }
        // sprintf(debugstring, "%s\tequates to %d", debugstring, (unsigned int)output_data_vector[braille_cell]);
        // ROS_INFO("%s", debugstring);
    }

    // print_output_data(output_data_vector);

    return output_data_vector;
}

void Mapper::print_output_data(std::vector<uint8_t> &data)
{
    // Test data size
    if (data.size() != 300)
    {
        ROS_ERROR("Printing output data but it's not 300 bytes!");
        return;
    }

    // const int linelength = 140;
    // char linebuffer[linelength] = "";

    // for (int i = 0; i < 300; i++)
    // {
    //     sprintf(linebuffer, "%s %3d", linebuffer, (unsigned int)data[i]);

    //     if (i != 0 && i % 30 == 0)
    //     {
    //         std::cout << linebuffer << '\n';
    //         memset(linebuffer, 0, linelength); // set buffer to 0
    //     }
    // }
}