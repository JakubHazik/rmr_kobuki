#include <utility>

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
//    data.convertTo(data, CV_16UC1);

//    data.convertTo(data, CV_16UC2); // potrebne iba teraz, ked mam debilny dataset
}


RobotMap::RobotMap(cv::Mat dataMatrix, int resolution):
        data(std::move(dataMatrix)), resolution(resolution) {
}


void RobotMap::saveToFile(std::string filename) {
//    data.convertTo(data, CV_16UC2);
//    cv::cvtColor(data, data, CV_16UC1);

    if (filename.find(".yaml") != string::npos) {
        cv::FileStorage fileStorage(filename, cv::FileStorage::WRITE);
        fileStorage << "resolution" << resolution;
        fileStorage << "data" << data;
    } else if (filename.find(".csv") != string::npos) {
        ofstream myfile;
        myfile.open(filename.c_str());
        myfile<< cv::format(data, cv::Formatter::FMT_CSV) << std::endl;
        myfile.close();
    }
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

MapSize RobotMap::getSize() {
    return {data.size().width, data.size().height};
}

cv::Mat RobotMap::getRawData() {
    return data;
}

int RobotMap::getResolution() {
    return resolution;
}

void RobotMap::setPointValue(MapPoint point, unsigned short value) {
//    data.at<cv::Vec>(Point(point.x, point.y))[mapLayer] = value;
    data.at<ushort>(Point(point.x, point.y))  = value;
}

unsigned short RobotMap::getPointValue(MapPoint point) {
    //return data.at<Vec2s>(Point(point.x, point.y))[mapLayer];
    return data.at<ushort>(Point(point.x, point.y));
}

RobotMap RobotMap::filterSpeckles() {
    //TODO filtrovanie neziaducich flakov na zaklade pravdepodobnosti
    return RobotMap("");
}

RobotMap RobotMap::getRobotMap() {
    return RobotMap(data, resolution);
}

bool RobotMap::containPoint(MapPoint point) {
    cv::Rect rect(cv::Point(), data.size());
    return rect.contains(cv::Point(point.x, point.y));
}

void RobotMap::showMap() {
    cv::Mat output;
    data.convertTo(output, CV_8UC1);
    resize(output, output, Size(), 5, 5);
    imshow("Robot map", output);
    cv::waitKey(0);
}
