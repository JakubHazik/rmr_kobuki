//
// Created by jakub on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_GLOBAL_PLANNER_H
#define KOBUKI_PROJECT_GLOBAL_PLANNER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "own_typedefs.h"
#include "robot_map.h"

using namespace std;

class GlobalPlanner {
public:
    explicit GlobalPlanner(RobotMap map);

    void setStartEndPose(RobotPose startPose, RobotPose goalPose);

    void floodFill();

private:
    MapPoint tfRealToMap(RobotPose realSpacePose);
    RobotPose tfMapToReal(MapPoint mapSpacePose);

    RobotMap map;

    MapPoint mapStartPose;
};


#endif //KOBUKI_PROJECT_GLOBAL_PLANNER_H