//
// Created by juerd on 20-3-23.
//

#ifndef TACTILEMAP_WITH_INTENT_MAPOVERLAY_H
#define TACTILEMAP_WITH_INTENT_MAPOVERLAY_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class MapOverlay(){
public:
    MapOverlay(Eigen::Vector2d& screen_size);
    ~MapOverlay();
    push(int location, int direction, int intensity);
    pinch(int location, int direction, int intensity);
    wave(int location, int direction, int intensity);

private:

}

#endif //TACTILEMAP_WITH_INTENT_MAPOVERLAY_H
