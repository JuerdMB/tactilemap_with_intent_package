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
    const char *filepath = "/home/juerd/catkin_ws/src/tactilemap_with_intent_package/images/test_map_1.png";
    cv::Mat image = cv::imread(filepath);

    int width = image.cols;
    int height = image.rows;

    ROS_INFO("Read image of size %dx%d, ", width, height);

    while (ros::ok())
    {
        if (width == 60 && height == 40)
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
            for(int x=0; x<width; x++){
                for(int y=0; y<height; y++){
                    uint8_t pixel = image.at<int>(y,x);
                    grid_map::Index index(x,y);
                    map.at(layername, index) = pixel;
                }
            }

            // Publish the map
            grid_map_msgs::GridMap gridmap_message;
            converter.toMessage(map, gridmap_message);
            output_image_publisher.publish(gridmap_message);
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