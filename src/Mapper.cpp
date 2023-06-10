#include "tactilemap_with_intent_package/Mapper.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <grid_map_msgs/GridMap.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

const float PI = 3.1415;

Mapper::Mapper(ros::NodeHandle nodeHandle, grid_map::GridMap &global_map) : nodeHandle_(nodeHandle)
{
    map_sub_ = nodeHandle_.subscribe(INPUT_MAP_TOPIC, 1, &Mapper::incomingMap, this);

    pub_bytes_dotpad = nodeHandle_.advertise<std_msgs::UInt8MultiArray>(TOPIC_DOTPAD_DATA, 1);
    pub_image_screensized = nodeHandle_.advertise<cv_bridge::CvImage>(TOPIC_IMAGE_SCREENSIZED, 1);
    pub_image_highres = nodeHandle_.advertise<cv_bridge::CvImage>(TOPIC_IMAGE_DETAILED, 1);

    // Initialize empty global GridMap
    this->globalMap_ = global_map;
    globalMap_.setBasicLayers(basicLayers);
    globalMap_.setFrameId("/map");

    receivedMap = false;

    if (!nodeHandle_.getParam("/tactilemap_with_intent/zoom_level", zoom_level))
        zoom_level = DEFAULT_ZOOM;
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
            if (!nodeHandle_.getParam("/tactilemap_with_intent/zoom_level", zoom_level))
            {
                return ZOOM_LOOKUP_ERROR;
            }

            // TODO get the zoom from parameter server
            float width = zoom_level * SCREEN_ASPECT_RATIO;
            float height = zoom_level;
            grid_map::Length length(height, width); // Waarom de F%*! krijg ik een landscape map met height & width omgewisseld

            // Try to get the current robot pose
            tf::StampedTransform robot_pose;
            odom_listener_.waitForTransform("/camera_link", "/map", ros::Time(0), ros::Duration(3.0));
            odom_listener_.lookupTransform("/camera_link", "/map", ros::Time(0), robot_pose);

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
            transformedZoomedMap = globalMap_.getTransformedMap(isometryTransform, STATICLAYER, "camera_link").getSubmap(grid_map::Position(0, 0), length, success);

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

// SENDING OUT THE MAPS

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

        ROS_INFO("Rescaled to screen resolution: from %2dx%2d (res=%.3f) to %2dx%2d (res=%.3f)",
                 transformedZoomedMap.getSize()(1), transformedZoomedMap.getSize()(0), transformedZoomedMap.getResolution(),
                 screensizedMap.getSize()(1), screensizedMap.getSize()(0), screensizedMap.getResolution());

        // PUBLISH MAP PREVIEWS
        cv_bridge::CvImage map_image_out;

        // HIGH RES TRANSFORMED
        map_image_out = get_image_from_map(transformedZoomedMap);
        pub_image_highres.publish(map_image_out);

        // SCREEN RESOLUTION PREVIEW
        map_image_out = get_image_from_map(screensizedMap);
        pub_image_screensized.publish(map_image_out);

        // // Convert the map to an array of bytes (uint8) and publish to message for dotpad receiver
        // std::vector<uint8_t> data = Mapper::getDataArray(screensizedMap, STATICLAYER);

        // if (data.size() != 0)
        // {
        //     std_msgs::UInt8MultiArray screendata_msg; // Construct the message
        //     screendata_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        //     screendata_msg.layout.dim[0].size = data.size();
        //     screendata_msg.layout.dim[0].stride = 1;
        //     screendata_msg.layout.dim[0].label = "data";
        //     screendata_msg.data.insert(screendata_msg.data.end(), data.begin(), data.end());

        //     pub_bytes_dotpad.publish(screendata_msg);
        //     // ROS_INFO("Published map data as a vector of size %d", screendata_msg.data.size());
        // }
        // else
        //     ROS_ERROR("Cannot publish empty vector");
    }
}

// STATIC HELPER FUNCTIONS
void Mapper::mapToScreenResolution(grid_map::GridMap &inputMap, grid_map::GridMap &outputMap)
{
    // Calculate the resolution for total size of 60x40 px
    double res = inputMap.getLength()(1) / SCREEN_WIDTH;

    // Implementation for resolution change
    grid_map::GridMapCvProcessing::changeResolution(inputMap, outputMap, res);
}

cv_bridge::CvImage Mapper::get_image_from_map(grid_map::GridMap &input_map)
{
    cv_bridge::CvImage outputImage;
    grid_map::GridMapRosConverter::toCvImage(input_map, STATICLAYER, sensor_msgs::image_encodings::RGB8, 0., 1., outputImage);

    // set current location (center) as red dot
    int numChannels = outputImage.image.channels();
    int width = outputImage.image.cols;
    int height = outputImage.image.rows;
    int centerLoc[2] = {width / 2, height / 2};

    // Loop through all pixels
    // double largestVal = 0.;
    // for (int y=0; y<height; y++)
    // {
    //     for (int x=0; x<width; x++)
    //     {
    //         for (int c=0; c<numChannels; c++)
    //         {
    //             cv::Vec3b & pixel = outputImage.image.at<cv::Vec3b>(cv::Point(y, x));

    //             ROS_INFO("%3d,%3d[%d] = %f", x, y, c, pixel[c]);

    //             // Test for highest val in Mat
    //             if (pixel[c] > largestVal)
    //                 largestVal = pixel[c];
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
        cv::Vec3b & pixel = outputImage.image.at<cv::Vec3b>(point);
        pixel[0] = 255.; // Blue channel
        pixel[1] = 0.;   // Green channel
        pixel[2] = 0.;   // Red channel
    }

    // ROS_INFO("Editing CvImage: size[wxh]= %3dx%3d, centerpixel=[%3d,%3d], channels=%d, largest value = %f", width, height, centerLoc[0], centerLoc[1], numChannels, largestVal);

    return outputImage;
}

std::vector<uint8_t> Mapper::getDataArray(grid_map::GridMap &inputMap, const std::string &layer)
{

    int mapWidth = inputMap.getSize()(0);
    int mapHeight = inputMap.getSize()(1);

    if (inputMap.getSize()(0) != 60)
    {
        ROS_ERROR("mapToDataArray: received map of incorrect dimensions %dx%d, returning empty vector", mapWidth, mapHeight);
        return {};
    }

    if (inputMap.getSize()(1) > 40)
        ROS_INFO("mapToDataArray: received map of correct width but incorrect height %d, cutting of last %d rows", mapHeight, mapHeight - 40);

    // IF the amount of vertical rows in the map < 40, we cannot fill the whole screen;
    // We can only loop through the

    int lastRow = 40;

    if (inputMap.getSize()(1) < 40)
    {
        ROS_INFO("mapToDataArray: input map insufficient length %d", mapHeight);
        lastRow = mapHeight;
    }

    std::vector<uint8_t> output_data_vector(300);                       // Empty vector of full screen size
    std::fill(output_data_vector.begin(), output_data_vector.end(), 0); // 0-initialize all values

    // Loop through 'braille cells' (collection of 8 map cells per braille cell), first by row, then by column

    for (int braille_cell = 0; braille_cell < 300; braille_cell++)
    {
        // Determine the start x & y values of this braille cell in the map
        int x_start = (braille_cell * 2) % 60;
        int y_start = ((braille_cell - braille_cell % 30) / 30) * 4;

        // Loop through each of the dots of this cell and write them to a bit in the output_data_vector
        for (int y = y_start; y < y_start + 4 && y < lastRow; y++)
        { // Loop through first column, then second

            for (int x = x_start; x < x_start + 2; x++)
            { // Loop through each of the for vertical rows per column

                grid_map::Index curIndex(x, y);
                uint8_t curPixel = inputMap.at(layer, curIndex);

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

                // Determine current bit position in byte, and push the bit into the byte
                uint8_t curBitLoc = (x - x_start) * 4 + (y - y_start); // If in second row, add 4 to y to obtain location
                output_data_vector[braille_cell] |= curPixel << (7 - curBitLoc);
            }
        }
    }

    return output_data_vector;
}
