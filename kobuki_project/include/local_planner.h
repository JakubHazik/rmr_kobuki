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
#include "config_defines.h"
#include <list>
#include <mutex>
#include <future>
#include <opencv2/opencv.hpp>
#include <syslog.h>


using namespace std;

class LocalPlanner {
public:
    LocalPlanner(RobotInterface *robotInterface, LidarInterface *lidarInterface);

    void processMovement(list<RobotPose> globalWaypoints);

    void stopMovement();

private:
    RobotInterface *robotInterface;
    LidarInterface *lidarInterface;

    /**
     * Compute collision on the robot way to goal position
     * @param localMap Map of local environment
     * @return True if way contain a collision
     */
    bool collisionCheck(RobotMap &localMap, RobotPose goalWaypoint);

    std::mutex waypoints_mtx;
    list<RobotPose> waypoints;

    list<RobotPose> computeBypass();

};


#endif //KOBUKI_PROJECT_LOCAL_PLANNER_H
