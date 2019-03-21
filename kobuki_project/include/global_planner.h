//
// Created by jakub on 14.3.2019.
//
/*
 * Usage:
 *   GlobalPlanner gPlanner(map, RobotPose{0,0}, RobotPose{-500, 500}, 20.5);
 *   auto way = gPlanner.getRobotWayPoints();      // get queue of waypoints in robot real space coordinates
 */

#ifndef KOBUKI_PROJECT_GLOBAL_PLANNER_H
#define KOBUKI_PROJECT_GLOBAL_PLANNER_H

#include <opencv2/opencv.hpp>
#include <queue>
#include <syslog.h>
#include "own_typedefs.h"
#include "robot_map.h"


using namespace std;

typedef struct {
    MapPoint mapPoint;
    unsigned short color;
} FloodPoint;

typedef MapPoint MapDirection;


class NoPathException : public std::exception {
public:
    explicit NoPathException(std::string msg): message(std::move(msg)) {}
    virtual const char* what() const noexcept {
        return message.c_str();
    }
private:
    string message;
};


class GlobalPlanner {

public:
    explicit GlobalPlanner(RobotMap map, RobotPose startPose, RobotPose goalPose, double robotWidth);

    /**
     * Function run flood fill algorithm, next find path and also find Robot waypoints
     * @return queue of RobotPose waypoints
     */
    queue<RobotPose> getRobotWayPoints();

    /**
     * Function call drawPathToMap and newt return this map
     * @return RobotMat with draw path in white color
     */
    RobotMap getRobotMapWithPath();

private:
    enum FFDirection{NONE, NORTH, NORTH_EAST, EAST, EAST_SOUTH, SOUTH, SOUTH_WEST, WEST, WEST_NORTH}; // !!! dont change it
    vector<MapDirection> directions = {{0,0},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};

    RobotMap map;

    vector<MapPoint> pathPoints;
    queue<RobotPose> robotWayPoints;

    MapPoint startPoint;
    MapPoint goalPoint;

    double robotWidth;

    // set start and goal point of robot
    void setStartEndPose(RobotPose startPose, RobotPose goalPose);

    // add boundaries around map
    void addWallBoundaries();

    // flood fill map with numbers
    void floodFill();

    /**
     * Function find next point based on fall gradient. Previous direction of gradient is prioritized.
     * @param point actual point around which we looking for fall gradient
     * @param direction gradient direction from previous point
     * @return
     */
    MapPoint findNextDirection(const MapPoint &point, FFDirection &direction);

    /*
     *  When map is flooded with floodFill() function, you can find path for robot.
     *  This function add points to @pathPoints and when direction of path is changed
     *  robot waypoint is added to @robotWayPoints.
     */
    void findPathPoints();

    /**
     * Move pixel map about offset in X, Y axes
     * @param map
     * @param direction offset in X, Y axes
     * @return translated map
     */
    cv::Mat translateMap(const cv::Mat &map, MapDirection direction);

    /*
     * Draw @pathPoints to map as white color
     */
    void drawPathToMap();

};


#endif //KOBUKI_PROJECT_GLOBAL_PLANNER_H
