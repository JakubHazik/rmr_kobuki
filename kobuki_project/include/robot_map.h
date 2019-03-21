//
// Created by jakub on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_ROBOT_MAP_H
#define KOBUKI_PROJECT_ROBOT_MAP_H

#include <string>
#include <fstream>
#include <exception>
#include <opencv2/opencv.hpp>
#include "own_typedefs.h"


class RobotMap {
public:
    RobotMap(MapSize mapSize, int resolution);

    RobotMap(std::string filename);

    RobotMap(cv::Mat dataMatrix, int resolution);

    ~RobotMap();

    void addMeasurement(RobotPose robotPose, LaserMeasurement *laserMeasurement);

    void saveToFile(std::string filename);

    MapSize getSize();

    int getResolution();

    cv::Mat getCVMatMap();

    void setPointValue(MapPoint point, unsigned short value);

    unsigned short getPointValue(MapPoint point);

    RobotMap filterSpeckles();

    RobotMap getRobotMap();

    void showMap();

    bool containPoint(MapPoint point);

    /**
     * Transform real robot pose from real 2D space to map space
     * @param realSpacePose robot pose
     * @return  map point
     */
    MapPoint tfRealToMap(RobotPose realSpacePose);

    /**
     * Transform map point to real 2D space robot pose
     * @param realSpacePose robot pose
     * @return  map point
     */
    RobotPose tfMapToReal(MapPoint mapSpacePose);

private:
    cv::Mat data;
    int resolution;     // resolution of one cell in [mm]
};


#endif //KOBUKI_PROJECT_ROBOT_MAP_H
