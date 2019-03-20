//
// Created by jakub on 14.3.2019.
//

#include "include/robot_map.h"
#include <exception>

using namespace std;
using namespace cv;

RobotMap::RobotMap(MapSize mapSize, int resolution): resolution(resolution) {
    data = Mat::zeros(mapSize.x, mapSize.y, CV_8UC1);
}

RobotMap::RobotMap(std::string filename) {
    cv::FileStorage fileStorage(filename, cv::FileStorage::READ);
    fileStorage["resolution"] >> resolution;
    fileStorage["data"] >> data;
}


RobotMap::RobotMap(cv::Mat dataMatrix, int resolution):
        data(std::move(dataMatrix)), resolution(resolution) {
}


void RobotMap::saveToFile(std::string filename) {
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

}

void RobotMap::addMeasurement(RobotPose robotPose, LaserMeasurement *laserMeasurement) {
    // TODO add measured data to map
}

MapPoint RobotMap::getSize() {
    return {data.rows, data.cols};
}

cv::Mat RobotMap::getRawData() {
    return data;
}

int RobotMap::getResolution() {
    return resolution;
}

void RobotMap::setPointValue(MapPoint point, unsigned short value) {
    try {
        data.at<ushort>(Point(point.x, point.y)) = value;
    }
    catch(...) {
        throw runtime_error("RobotMap: Set point is out of range");
    }
}

unsigned short RobotMap::getPointValue(MapPoint point) {
    try {
        return data.at<ushort>(Point(point.x, point.y));
    } catch(...) {
        throw runtime_error("RobotMap: Get point is out of range");
    }
}

RobotMap RobotMap::filterSpeckles() {
    //TODO filtrovanie neziaducich flakov na zaklade pravdepodobnosti
    return RobotMap("");
}

RobotMap RobotMap::getRobotMap() {
    // TODO filter map probability, return map only as 0, 1
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
