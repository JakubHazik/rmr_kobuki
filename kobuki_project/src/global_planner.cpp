//
// Created by jakub on 14.3.2019.
//

#include <include/global_planner.h>

#include "include/global_planner.h"

GlobalPlanner::GlobalPlanner(RobotMap map): map(map) {

}

void GlobalPlanner::setStartEndPose(RobotPose startPose, RobotPose goalPose) {
    // save start pose to class member
    mapStartPose = tfRealToMap(startPose);

    // save goal pose map as number 2
    MapPoint mapGoalPose = tfRealToMap(goalPose);
    map.setPointValue(mapGoalPose, RobotMap::MEASUREMENT, 2);
}

MapPoint GlobalPlanner::tfRealToMap(RobotPose realSpacePose) {
    MapPoint mapPose;
    mapPose.x = (int)round(realSpacePose.x / map.getResolution());
    mapPose.y = (int)round(realSpacePose.y / map.getResolution());
    return mapPose;
}

RobotPose GlobalPlanner::tfMapToReal(MapPoint mapSpacePose) {
    RobotPose realSpacePose;
    realSpacePose.x = mapSpacePose.x * map.getResolution();
    realSpacePose.y = mapSpacePose.y * map.getResolution();
    return realSpacePose;
}

void GlobalPlanner::floodFill() {

}
