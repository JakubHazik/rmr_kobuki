//
// Created by jakub on 14.3.2019.
//

#include "include/global_planner.h"

GlobalPlanner::GlobalPlanner(RobotMap map, RobotPose startPose, RobotPose goalPose): map(map) {
    setStartEndPose(startPose, goalPose);
}


void GlobalPlanner::setStartEndPose(RobotPose startPose, RobotPose goalPose) {
    // save start pose to class member
    this->startPoint = tfRealToMap(startPose);

    // save goal pose map as number 2
    this->goalPoint = tfRealToMap(goalPose);
}

MapPoint GlobalPlanner::tfRealToMap(RobotPose realSpacePose) {
    auto mapSize = map.getSize();
    MapPoint mapPose;
    mapPose.x = (int)round(realSpacePose.x / map.getResolution()) + mapSize.x / 2;
    mapPose.y = (int)round(realSpacePose.y / map.getResolution()) + mapSize.y / 2;
    return mapPose;
}

RobotPose GlobalPlanner::tfMapToReal(MapPoint mapSpacePose) {
    auto mapSize = map.getSize();
    RobotPose realSpacePose;
    realSpacePose.x = (mapSpacePose.x - mapSize.x / 2) * map.getResolution();
    realSpacePose.y = (mapSpacePose.y - mapSize.y / 2) * map.getResolution();
    return realSpacePose;
}

void GlobalPlanner::floodFill() {
    typedef struct {
        MapPoint mapPoint;
        unsigned short color;
    } FloodPoint;

    queue<FloodPoint> floodPoints;

    // add first (goal) point, it has value = 2
    floodPoints.push({goalPoint, 2});

    while (!floodPoints.empty()) {
        auto floodPoint = floodPoints.front();
        floodPoints.pop();

        if (!map.containPoint(floodPoint.mapPoint) || map.getPointValue(floodPoint.mapPoint) != 0) {
            continue;
        }

        map.setPointValue(floodPoint.mapPoint, floodPoint.color);

        floodPoints.push({{floodPoint.mapPoint.x + 1, floodPoint.mapPoint.y + 1}, floodPoint.color + 1});
        floodPoints.push({{floodPoint.mapPoint.x + 1, floodPoint.mapPoint.y    }, floodPoint.color + 1});
        floodPoints.push({{floodPoint.mapPoint.x + 1, floodPoint.mapPoint.y - 1}, floodPoint.color + 1});
        floodPoints.push({{floodPoint.mapPoint.x    , floodPoint.mapPoint.y - 1}, floodPoint.color + 1});
        floodPoints.push({{floodPoint.mapPoint.x - 1, floodPoint.mapPoint.y - 1}, floodPoint.color + 1});
        floodPoints.push({{floodPoint.mapPoint.x - 1, floodPoint.mapPoint.y    }, floodPoint.color + 1});
        floodPoints.push({{floodPoint.mapPoint.x - 1, floodPoint.mapPoint.y + 1}, floodPoint.color + 1});
        floodPoints.push({{floodPoint.mapPoint.x    , floodPoint.mapPoint.y + 1}, floodPoint.color + 1});
    }
}

