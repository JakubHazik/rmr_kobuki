//
// Created by jakub on 28.3.2019.
//

#ifndef KOBUKI_PROJECT_VISUALIZER_H
#define KOBUKI_PROJECT_VISUALIZER_H

#include "include/robot_map.h"
#include "include/own_typedefs.h"
#include <opencv2/opencv.hpp>

/*
 * prazdny priestor biela
 * steny cierna
 * waipoints modra
 * robot crevena
 * floodfill zlta
 * path zelena
 *
 *
 */

// COLOR VALUES Scalar(Blue, Green, Red)
#define COLOR_WHITE Scalar(255,255,255)
#define COLOR_RED Scalar(0,0,255)
#define COLOR_GREEN Scalar(0,255,0)
#define COLOR_BLUE Scalar(255,0,0)
#define COLOR_BLACK Scalar(1,1,1)


class Visualizer {
public:
    Visualizer(MapSize mapSize, int robotWidth, int mapResolution);

    cv::Mat getImage(RobotPose robotPose);
    cv::Mat getImage(RobotPose robotPose, RobotMap laserData);
    cv::Mat getImage(RobotPose robotPose, RobotMap environmentMap, RobotMap laserData);

    RobotPose robotPose = {};
    RobotPose goalPose = {};
    cv::Mat environmentMap;
    cv::Mat path;
    cv::Mat waypoints;
    cv::Mat floodFill;
private:
    void addRobot(cv::Mat &inputOutput, RobotPose robotPose, cv::Scalar color);

    void addFloodFill(cv::Mat &output, cv::Mat &addImg);

//    cv::Mat robotImage;

    void addImageMask(cv::Mat &output, cv::Mat &addImg, cv::Scalar color);

    void convertImage(cv::Mat &img);

    MapSize mapSize;

    cv::Mat robotImage, robotImageMask;

    int mapResolution;
    int robotWidth;




};


#endif //KOBUKI_PROJECT_VISUALIZER_H
