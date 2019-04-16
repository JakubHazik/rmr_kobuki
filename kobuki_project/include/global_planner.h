//
// Created by jakub on 14.3.2019.
//
/*
 * Computation speed:
 *  Measured searching goal across whole idea (map resolution=50):
 *   -O4 : 14 ms
 *   -O2: 14 ms
 *   without O: 20 ms
 *
 * Usage:
    RobotMap map("../ideal_map.txt", true, RobotPose{500,500});

    Visualizer vis(map.getSize(), Kconfig::HW::ROBOT_WIDTH, Kconfig::Defaults::MAP_RESOLUTION);
    auto odom = RobotPose{0,0};
    auto goal = RobotPose{4500, 3500};

    GlobalPlanner gPlanner(map, odom, goal, Kconfig::HW::ROBOT_WIDTH);
    auto way = gPlanner.getRobotWayPoints();

    vis.environmentMap = map.getCVMatMap();
    vis.waypoints = gPlanner.getWayPointsImage();
    vis.floodFill = gPlanner.getFloodFillImage();
    vis.path = gPlanner.getPathImage();
    auto img = vis.getImage(odom);
    cv::imshow("img", img);
    cv::waitKey(0);
 */

#ifndef KOBUKI_PROJECT_GLOBAL_PLANNER_H
#define KOBUKI_PROJECT_GLOBAL_PLANNER_H

#include <opencv2/opencv.hpp>
#include <queue>
#include <syslog.h>
#include "own_typedefs.h"
#include "robot_map.h"
#include <list>


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
    /**
     *
     * @param map
     * @param startPose
     * @param goalPose
     * @param robotWidth Robot width in mm
     */
    explicit GlobalPlanner(RobotMap map, RobotPose startPose, RobotPose goalPose, int robotWidth);

    /**
     * Function run flood fill algorithm, next find path and also find Robot waypoints
     * @return queue of RobotPose waypoints
     */
    list<RobotPose> getRobotWayPoints();

//    /**
//     * Function call drawPathToMap and newt return this map
//     * @return RobotMat with draw path in white color
//     */
//    RobotMap getRobotMapWithPath();

    cv::Mat getPathImage();
    cv::Mat getWayPointsImage();
    cv::Mat getFloodFillImage();

private:
    enum FFDirection{NONE, NORTH, NORTH_EAST, EAST, EAST_SOUTH, SOUTH, SOUTH_WEST, WEST, WEST_NORTH}; // !!! dont change it
    vector<MapDirection> directions = {{0,0},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};

    RobotMap map;
    RobotMap floodMap;
    RobotMap pathMap;
    RobotMap wayPoints;

    vector<MapPoint> pathPoints;
    list<RobotPose> robotWayPoints;

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
