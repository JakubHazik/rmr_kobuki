//
// Created by jakub on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_ROBOT_MAP_H
#define KOBUKI_PROJECT_ROBOT_MAP_H

#include <string>
#include <fstream>
#include <syslog.h>
#include <exception>
#include <opencv2/opencv.hpp>
#include "own_typedefs.h"
#include "config_defines.h"

#define SHOW_IMAGE_SCALE_FACTOR 2

class RobotMap {
public:
    /**
     * @param mapSize size of whole map in [mm]
     * @param resolution resolution of map in [mm]
     */
    RobotMap(RobotPose mapSize, int resolution);

    /**
     * @param filename name of file from filesystem
     */
    RobotMap(std::string filename, bool idealMap = false, RobotPose robotReference = {0});

    RobotMap(cv::Mat dataMatrix, int resolution);

    /**
     * Copy constructor
     * @param robotMap
     */
    RobotMap(const RobotMap &robotMap);

    ~RobotMap();

    /**
     * Adding current set of measuremets from lidar and odometry to enviroment Map
     *
     * Working with incrementation principe - increment points in map where obstacles were found
     *
     * @param - robotPose current position from odometry
     * @param - laserMeasurement set of measurements from lidar
     */
    void addMeasurement(RobotPose robotPose, LaserMeasurement *laserMeasurement, unsigned short _pointValue = 2);

    /**
     * Adding current set of measuremets from lidar and odometry to enviroment Map
     *
     * Implemented with forgetting factor. _pointValue is used as maximal value of point in map and also as n.o. iterations to fully get rid off measurement from map.
     *
     * In each iteration is whole map decremented by 1, that means the smaller value point in map goes, the older is the measurement.
     * Every cycle is the newest measurement written to map with given maximal value of point.
     *
     * @param - robotPose current position from odometry
     * @param - laserMeasurement set of measurements from lidar
     * @param _pointValue defined constant of maximal value for new point
     */
    void addMeasurementForgetting(RobotPose robotPose, LaserMeasurement *laserMeasurement, unsigned short _pointValue = 2);

    /**
     * Save Map class to .yaml file for future purposes
     * @param filename - name of saved file
     */
    void saveToFile(std::string filename);

    /**
     * Get map size
     * @return MapPoint (x,y) resolution (or rows,cols)
     */
    MapSize getSize();

    /**
     * Get map resolution (one square dimensions)
     * @return resolution
     */
    int getResolution();

    /**
     * Get matrix as set of zeros and ones, where 1 represents obstacle
     * Uses filterSpeckles with given threshold value to filter lidar noise
     * @param threshold_value lidar noise filter
     * @return cv::Mat object
     */
    cv::Mat getCVMatMap();

    /**
     * Setter for given point (index, pixel) value
     * @param point given point to be set to
     * @param value vale to be set to
     */
    void setPointValue(MapPoint point, unsigned short value);

    unsigned short getPointValue(MapPoint point);

    /**
     * Filter measured dataset with given threshold value
     * @param threshold_value lidar noise filter
     */
    void filterSpeckles();

    RobotMap getRobotMap();

    void showMap();

    /**
     * Set whole map to zeros, clear all data for new measurement
     */
    void clearMap();

    /**
     * Check if Map contains given point
     * @param point given point
     * @return true if Map contains a point
     */
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

    /**
     * Transform real robot pose from real 2D space to map space
     * @param realSpacePose robot pose
     * @return  map point
     */
    static MapPoint tfRealToMap(RobotPose realSpacePose, MapSize mapSize, int resolution);

    /**
     * Transform map point to real 2D space robot pose
     * @param realSpacePose robot pose
     * @return  map point
     */
    static RobotPose tfMapToReal(MapPoint mapSpacePose, MapSize mapSize, int resolution);

    void translateMap(RobotPose offset);

    void rotateMap(double angle, RobotPose center = {});

private:
    cv::Mat data;
    cv::Mat outputMap;
    int resolution;     // mapResolution of one cell in [mm]
    void printWallToMap(const std::vector<MapPoint> &corners);

    void loadIdealMap(std::string filename);
};


#endif //KOBUKI_PROJECT_ROBOT_MAP_H
