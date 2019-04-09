//
// Created by jakub on 28.3.2019.
//
/*
 Usage:
    RobotMap map(RobotPose{6000 * 2, 6000 * 2}, 30);
    map.loadIdealMap("../ideal_map", RobotPose{5300, 600, 180});

    int robotWidth = 260; //mm
    RobotPose odom = {0,0,45};
    RobotPose goal = {5000, -1600, 0};
    GlobalPlanner gPlanner(map.getRobotMap(), odom, goal, robotWidth);

    Visualizer visualizer(map.getSize(), robotWidth, map.getResolution());
    visualizer.environmentMap = map.getCVMatMap();
    visualizer.path = gPlanner.getPathImage();
    visualizer.waypoints = gPlanner.getWayPointsImage();
    visualizer.floodFill = gPlanner.getFloodFillImage();

    auto img = visualizer.getImage(odom);

    cv::resize(img, img, cv::Size(600,600));
    cv::imshow("a", img);
    cv::waitKey(0);
 */

#ifndef KOBUKI_PROJECT_VISUALIZER_H
#define KOBUKI_PROJECT_VISUALIZER_H

#include "robot_map.h"
#include "own_typedefs.h"
#include "config_defines.h"
#include <opencv2/opencv.hpp>

class Visualizer {
public:
    Visualizer(MapSize mapSize, int robotWidth, int mapResolution);

    cv::Mat getImage(RobotPose robotPose);                      // TODO nemusime vracat Mat, mozeme rovno QCanvas
    cv::Mat getImage(RobotPose robotPose, RobotMap laserData);  //TODO implementovat podla svojej lubovole

    cv::Mat environmentMap;
    cv::Mat path;
    cv::Mat waypoints;
    cv::Mat floodFill;
private:
    MapSize mapSize;
    int mapResolution;
    int robotWidth;

    /*
     * Draw robot to image
     */
    void addRobot(cv::Mat &inputOutput, RobotPose robotPose, cv::Scalar color);

    /*
     * Draw flood fill to image
     */
    void addFloodFill(cv::Mat &output, cv::Mat &addImg);

    /**
     * Add a image to output image as next no transparency layer
     * @param output output image
     * @param addImg image which we want to add, this Mat can have only <0,1> values
     * @param color color
     */
    void addImageMask(cv::Mat &output, cv::Mat &addImg, cv::Scalar color);
};


#endif //KOBUKI_PROJECT_VISUALIZER_H
