#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <bits/stdc++.h>
#include "tactilemap_with_intent_package/MapDataConverter.h"
#include "tactilemap_with_intent_package/Mapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");
    ros::Publisher output_data_publisher = nodeHandle.advertise<std_msgs::UInt8MultiArray>(OUTPUT_DATA_TOPIC, 1);
    ros::Publisher output_image_publisher = nodeHandle.advertise<grid_map_msgs::GridMap>(OUTPUT_PREVIEWIMG_TOPIC, 1);
    grid_map::GridMapRosConverter converter;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0.0);
    transform.setRotation(q);

    ros::Rate rate(.5);

    // Load images and transform into image
    const char *filepath = "/home/juerd/catkin_ws/src/tactilemap_with_intent_package/images/grid_1.png";
    cv::Mat image = cv::imread(filepath, cv::IMREAD_GRAYSCALE);   //, cv::IMREAD_REDUCED_GRAYSCALE_8

    ROS_INFO("Read image size= %dx%d", image.cols, image.rows);

    while (ros::ok())
    {
        if (image.cols == 60 && image.rows == 40)
        {
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

            // Create map from image
            std::string layername = "image";
            grid_map::GridMap map({layername});
            map.setFrameId("world");
            grid_map::Length length(60, 40);
            map.setGeometry(length, 1., grid_map::Position(0));
            ROS_INFO("Created map of %2dx%2d cells", map.getSize()(0), map.getSize()(1));

            // Write the image to the map
            for (int row = 0; row < image.rows; row++)
            {
                for (int col = 0; col < image.cols; col++)
                {
                    uint8_t pixelValue = image.at<double>(row, col);
                    grid_map::Index index(col, row);
                    map.at(layername, index) = pixelValue;
                    ROS_INFO("%3d,%3d = %d", col, row, pixelValue);
                }
            }

            // Publish the preview
            grid_map_msgs::GridMap preview_message;
            converter.toMessage(map, preview_message);
            output_image_publisher.publish(preview_message);
            ROS_INFO("Published image to %s", OUTPUT_PREVIEWIMG_TOPIC.c_str());

            // Publish the images
            std::vector<uint8_t> data = MapDataConverter::mapToDataArray(map, layername);
            std_msgs::UInt8MultiArray screendata_msg; // Construct the message
            screendata_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            screendata_msg.layout.dim[0].size = data.size();
            screendata_msg.layout.dim[0].stride = 1;
            screendata_msg.layout.dim[0].label = "data";
            screendata_msg.data.insert(screendata_msg.data.end(), data.begin(), data.end());
            output_data_publisher.publish(screendata_msg);
            ROS_INFO("Published Message to %s", OUTPUT_DATA_TOPIC.c_str());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};