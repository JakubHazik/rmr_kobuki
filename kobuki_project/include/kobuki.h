//
// Created by martin on 21.3.2019.
//

#ifndef KOBUKI_PROJECT_KOBUKI_H
#define KOBUKI_PROJECT_KOBUKI_H

#include "own_typedefs.h"
#include "robot_interface.h"
#include "lidar_interface.h"
#include "robot_map.h"
#include "local_planner.h"
#include "global_planner.h"
#include "visualizer.h"
#include "config_defines.h"

#include <list>
#include <opencv2/opencv.hpp>
#include <exception>


using namespace std;

class Kobuki {
public:
    explicit Kobuki();

    virtual ~Kobuki();

    cv::Mat getEnvironmentAsImage(bool environment, bool waypoints, bool path, bool floodFill, bool laserScan);

    void loadMapFromFile(string filepath);

    void saveMapToFile(string filepath);

    void clearMap();

    RobotPose getRobotPosition();

    void setRobotActualPosition(double x, double y, double fi);

    void moveRobotToPosition(double x, double y, SPACE space);

    /**
     * This also clear all waypoints in local planner
     */
    void stopRobotMovement();

//    void updateGlobalMap();  /// todo toto je riadna ******

private:

    RobotInterface *robotInterface;
    LidarInterface *lidarInterface;
    LocalPlanner *lPlanner;

    RobotMap map;

    cv::Mat gPlannerFloodFill;
    cv::Mat gPlannerWaypoints;
    cv::Mat gPlannerPath;
};

#endif //KOBUKI_PROJECT_KOBUKI_H