//
// Created by juerd on 20-3-23.
//

#ifndef TACTILEMAP_WITH_INTENT_MAPOVERLAY_H
#define TACTILEMAP_WITH_INTENT_MAPOVERLAY_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class MapOverlay {
public:
    MapOverlay();

    ~MapOverlay();

    void push(int location, int direction, int intensity);

    void pinch(int location, int direction, int intensity);

    void wave(int location, int direction, int intensity);


};

#endif //TACTILEMAP_WITH_INTENT_MAPOVERLAY_H
