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

#define SHOW_IMAGE_SCALE_FACTOR 2

class RobotMap {
public:
    /**
     *
     * @param mapSize size of whole map in [mm]
     * @param resolution resolution of map in [mm]
     */
    RobotMap(RobotPose mapSize, int resolution);

    RobotMap(std::string filename);

    RobotMap(cv::Mat dataMatrix, int resolution);

    ~RobotMap();

    void loadIdealMap(std::string filename, RobotPose robotReference);

    void addMeasurement(RobotPose robotPose, LaserMeasurement *laserMeasurement);

    void saveToFile(std::string filename);

    MapSize getSize();

    int getResolution();

    cv::Mat getCVMatMap(int threshold_value);

    void setPointValue(MapPoint point, unsigned short value);

    unsigned short getPointValue(MapPoint point);

    void filterSpeckles(int threshold_value);

    RobotMap getRobotMap();

    void showMap();

    void clearMap();

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
    cv::Mat outputMap;
    int resolution;     // resolution of one cell in [mm]
    void printWallToMap(const std::vector<MapPoint> &corners);
    void translateMap(MapPoint direction);
};


#endif //KOBUKI_PROJECT_ROBOT_MAP_H
