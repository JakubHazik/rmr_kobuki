//
// Created by jakub on 31.3.2019.
//

#ifndef KOBUKI_PROJECT_LOCAL_PLANNER_H
#define KOBUKI_PROJECT_LOCAL_PLANNER_H

#include "robot_interface.h"
#include "robot_map.h"
#include "lidar_interface.h"
#include "own_typedefs.h"
#include "global_planner.h"
#include <list>
#include <mutex>
#include <condition_variable>
#include <boost/bind.hpp>
#include <thread>


#define GOAL_ZONE_DISTANCE 300

using namespace std;

class LocalPlanner {
public:
    LocalPlanner(RobotInterface *robotInterface, LidarInterface *lidarInterface, list<RobotPose> globalWaypoints);

    void processMovement();

//    void goToGoal(RobotPose goalPose);

private:
    RobotInterface *robotInterface;
    LidarInterface *lidarInterface;

//    mutex goalAchieved_mtx;
//    condition_variable goalAchieved;
//    RobotPose goalPose;
//
//    mutex waypoints_mtx;
    list<RobotPose> waypoints;

//    void wayPointZoneAchieved_cbk();

    list<RobotPose> computeBypass();        //TODO implement

};


#endif //KOBUKI_PROJECT_LOCAL_PLANNER_H
