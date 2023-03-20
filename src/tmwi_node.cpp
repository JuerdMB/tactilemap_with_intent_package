//
// Created by juerd on 16-3-23.
//

#include <ros/ros.h>
#include <tactilemap_with_intent/Mapper.h>
#include <tactilemap_with_intent/MapOverlay.h>

// Create global maps
grid_map::GridMap globalmap(basicLayers);
grid_map::GridMap tfmap(basicLayers);
Mapper mapper(nodeHandle, globalmap, tfmap);

void publishScreen(){
    mapper.updateSubMap();
    CVimage mapImg = mapper.getMapImg();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    ros::Timer timer = nodeHandle.createTimer(ros::Duration(0.05), publishScreen());

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};