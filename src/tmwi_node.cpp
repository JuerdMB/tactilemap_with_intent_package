//
// Created by juerd on 16-3-23.
//

#include <ros/ros.h>
#include <tactilemap_with_intent/Mapper.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    // Create global maps
    grid_map::GridMap globalmap(basicLayers);
    grid_map::GridMap tfmap(basicLayers);
    Mapper mapper(nodeHandle, globalmap, tfmap);

    ros::Rate rate(30); //Spin every 30 hz

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};