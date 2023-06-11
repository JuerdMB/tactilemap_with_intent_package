#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <bits/stdc++.h>
#include "tactilemap_with_intent_package/Mapper.h"
#include <filesystem>

const std::string imageLayer = "image";

// namespace fs = std::filesystem;

void getMapFromImage(cv::Mat &inputimage, grid_map::GridMap &inputmap)
{
    // Create map from image
    inputmap.setFrameId("world");

    int lengthx = inputimage.rows;  // X is verticale richting!!!
    int lengthy = inputimage.cols; 
    grid_map::Length length(lengthx, lengthy);
    inputmap.setGeometry(length, 1., grid_map::Position(0));

    // Write the image to the map
    for (int y = 0; y < inputimage.rows; y++)
    {
        for (int x = 0; x < inputimage.cols; x++)
        {
            cv::Vec3b bgrPixel = inputimage.at<cv::Vec3b>(y, x);
            float avg = (bgrPixel.val[0] + bgrPixel.val[1] + bgrPixel.val[2]) / 3.;
            uint8_t pixelVal = avg > 128.;

            grid_map::Index index(y, x);
            inputmap.at(imageLayer, index) = pixelVal;
            // ROS_INFO("%3d,%3d:\t b=%3d g=%3d r=%3d \t = %d", col, row, bgrPixel.val[0], bgrPixel.val[1], bgrPixel.val[2], pixelVal);
        }
    }
}

void testPrintMap(grid_map::GridMap &inputmap)
{
    ros::V_string layers = inputmap.getBasicLayers();
    grid_map::Matrix &mapdata = inputmap[layers[1]];
    for (grid_map::GridMapIterator iterator(inputmap); !iterator.isPastEnd(); ++iterator)
    {
        const size_t i = iterator.getLinearIndex();
        ROS_INFO("%3ld=\t %f", i, mapdata(i));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Tactilemap_tester");
    ros::NodeHandle nodeHandle("~");
    ros::Publisher pub_dotpad_data = nodeHandle.advertise<std_msgs::UInt8MultiArray>(TOPIC_DOTPAD_DATA, 1);
    ros::Publisher pub_map_preview = nodeHandle.advertise<grid_map_msgs::GridMap>(TOPIC_MAP_SCREENSIZED, 1);
    ros::Publisher pub_image_preview = nodeHandle.advertise<cv_bridge::CvImage>(TOPIC_IMAGE_DETAILED, 1);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0.0);
    transform.setRotation(q);

    grid_map::GridMapRosConverter converter;

    while (ros::ok())
    {
        std::string directory = "/home/juerd/catkin_ws/src/tactilemap_with_intent_package/images/";
        cv::Mat image;

        // Try to read image
        do
        {
            std::cout << "Choose file from /images: ";

            // Get filename of image to print
            std::string filename_in;
            std::cin.clear();
            std::cin.ignore(INT_MAX, '\n');
            std::cin >> filename_in;
            // std::getline(std::cin, filename_in);
            cv::String filepath = directory + filename_in;
            ROS_INFO("Trying to read image from: %s", filepath.c_str());
            image = cv::imread(filepath);

            if (image.empty())
                ROS_ERROR("That image does not exist");

        } while (image.empty());

        if (image.cols != 60 || image.rows != 40)
        {
            // Incorrect image size
            ROS_INFO("Incorrect image size: %d x %d", image.cols, image.rows);
            break;
        }
        ROS_INFO("Succesfully loaded image: %dx%d", image.cols, image.rows);

        grid_map::GridMap map({imageLayer});
        getMapFromImage(image, map);
        ROS_INFO("Transformed image into map of %2dx%2d cells", map.getSize()(1), map.getSize()(0));    // getSize()(0) = y-waarde, getSize()(1) = x-waarde
        // testPrintMap(map);

        // Publish the preview image as map for RVIZ display
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
        grid_map_msgs::GridMap gridmap_preview_message;
        converter.toMessage(map, gridmap_preview_message);
        pub_map_preview.publish(gridmap_preview_message);
        ROS_INFO("Published image to %s", TOPIC_MAP_SCREENSIZED.c_str());

        // Publish as CV Image
        cv_bridge::CvImage map_image_out;
        map_image_out = Mapper::get_image_from_map(map, imageLayer);
        pub_image_preview.publish(map_image_out);
        ROS_INFO("Published image to %s", TOPIC_IMAGE_DETAILED.c_str());

        // Publish the data
        std::vector<uint8_t> data = Mapper::getDataArray(map, imageLayer);
        std_msgs::UInt8MultiArray screendata_msg; // Construct the message
        screendata_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        screendata_msg.layout.dim[0].size = data.size();
        screendata_msg.layout.dim[0].stride = 1;
        screendata_msg.layout.dim[0].label = "data";
        screendata_msg.data.insert(screendata_msg.data.end(), data.begin(), data.end());
        pub_dotpad_data.publish(screendata_msg);
        ROS_INFO("Published data to %s", TOPIC_DOTPAD_DATA.c_str());

        ros::spinOnce();
    }

    return 0;
};