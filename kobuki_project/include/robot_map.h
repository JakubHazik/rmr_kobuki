//
// Created by jakub on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_ROBOT_MAP_H
#define KOBUKI_PROJECT_ROBOT_MAP_H

#include "own_typedefs.h"

#include <string>
#include <fstream>
#include <exception>
#include <opencv2/opencv.hpp>
#include <fstream>

// TODO classu treba doimplementovat podla potreby
/*
 * Reprezentacia mapy:
 * matica uchovavajuca mapu (RobotMap::data) ma 2 vrstvy:
 *              MEASUREMENT - namerane body lidarom
 *              PROBABILITY - pravdepodobnostna charakteristika merani
 */

class RobotMap {
public:
//    enum MapLayer {MEASUREMENT, PROBABILITY};

    RobotMap(MapSize mapSize, int resolution);
    RobotMap(std::string filename);
    RobotMap(cv::Mat dataMatrix, int resolution);
    ~RobotMap();

    void addMeasurement(RobotPose robotPose, LaserMeasurement *laserMeasurement);

    void saveToFile(std::string filename);

    MapSize getSize();

    int getResolution();

    cv::Mat getRawData();

    void setPointValue(MapPoint point, unsigned short value);

    unsigned short getPointValue(MapPoint point);

    RobotMap filterSpeckles();

    RobotMap getRobotMap();

    void showMap();

    bool containPoint(MapPoint point);
private:
    cv::Mat data;

//    int **data;          // keep measured data (map representation)
//    int **probability;   // keep likelihood/ probability of measured data

//    MapSize mapSize;
    int resolution;     // resolution of one cell in [mm]
    int rows, cols;

//    void allocateMatrix(int **array, int Xsize, int Ysize);
//    void deallocMatrix(int **array, int Ysize);
};


#endif //KOBUKI_PROJECT_ROBOT_MAP_H
