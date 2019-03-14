//
// Created by jakub on 14.3.2019.
//

#include <include/robot_map.h>

#include "include/robot_map.h"

using namespace std;
using namespace cv;

RobotMap::RobotMap(MapSize mapSize, int resolution): resolution(resolution) {

//    mapSize.x = Xsize;
//    mapSize.y = Ysize;
    //TODO
//    allocateMatrix(data, Xsize, Ysize);
//    allocateMatrix(probability, Xsize, Ysize);

}

RobotMap::RobotMap(std::string filename) {
    cv::FileStorage fileStorage(filename, cv::FileStorage::READ);
    fileStorage["resolution"] >> resolution;
    fileStorage["data"] >> data;

    data.convertTo(data, CV_16UC2); // potrebne iba teraz, ked mam debilny dataset
}

void RobotMap::saveToFile(std::string filename) {
//    data.convertTo(data, CV_16UC2);
//    cv::cvtColor(data, data, CV_16UC1);
    cv::FileStorage fileStorage(filename, cv::FileStorage::WRITE);
    fileStorage << "resolution" << resolution;
    fileStorage << "data" << data;
}

RobotMap::~RobotMap() {
//    deallocMatrix(data, mapSize.y);
//    deallocMatrix(probability, mapSize.y);
}


//void RobotMap::allocateMatrix(int **array, int Xsize, int Ysize) {
//    array = new int*[Ysize];
//    for(int i = 0; i < Ysize; ++i) {
//        array[i] = new int[Xsize];
//    }
//}
//
//void RobotMap::deallocMatrix(int **array, int Ysize) {
//    for(int i = 0; i < Ysize; ++i)
//        delete [] array[i];
//    delete [] array;
//
//}

void RobotMap::addMeasurement(RobotPose robotPose, LaserMeasurement *laserMeasurement) {
    // TODO add measured data to map
}

MapPoint RobotMap::getSize() {
    return {data.size().width, data.size().height};
}

cv::Mat RobotMap::getRawData() {
    return data;
}

int RobotMap::getResolution() {
    return resolution;
}

void RobotMap::setPointValue(MapPoint point, MapLayer mapLayer, unsigned short value) {
    data.at<cv::Vec2s>(Point(point.x, point.y))[mapLayer] = value;
}

int RobotMap::getPointValue(MapPoint point, MapLayer mapLayer) {
    return data.at<Vec2s>(Point(point.x, point.y))[mapLayer];
}

RobotMap RobotMap::filterSpeckles() {
    //TODO filtrovanie neziaducich flakov na zaklade PROBABILITY vrstvy
    return RobotMap({}, 0);
}


