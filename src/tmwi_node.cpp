#include <ros/ros.h>
#include <tactilemap_with_intent/Mapper.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "Tactilemap_with_intent");
    ros::NodeHandle nodeHandle("~");

    // Create global maps
    grid_map::GridMap globalmap(basicLayers);
    grid_map::GridMap transformedmap(basicLayers);
    Mapper mapper(nodeHandle, globalmap, transformedmap);

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
};