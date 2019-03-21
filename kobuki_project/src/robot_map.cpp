//
// Created by jakub on 14.3.2019.
//

#include "include/robot_map.h"
#include <limits>

using namespace std;
using namespace cv;

RobotMap::RobotMap(MapSize mapSize, int resolution): resolution(resolution) {
    data = Mat::zeros(mapSize.x, mapSize.y, CV_16UC1);  // construct matrix with unsigned short values
}

RobotMap::RobotMap(std::string filename) {
    cv::FileStorage fileStorage(filename, cv::FileStorage::READ);
    fileStorage["resolution"] >> resolution;
    fileStorage["data"] >> data;
    data.convertTo(data, CV_16UC1); // convert input matrix to unsigned short values
}

RobotMap::RobotMap(cv::Mat dataMatrix, int resolution): resolution(resolution) {
    dataMatrix.convertTo(data, CV_16UC1);   // convert input matrix to unsigned short values
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
    } else {
        throw invalid_argument("RobotMap: saveToFile: incorrect file extension");
    }
}

RobotMap::~RobotMap() {

}

void RobotMap::addMeasurement(RobotPose robotPose, LaserMeasurement *laserMeasurement) {
    cout << "odom: " << robotPose.x << "  " << robotPose.y << "  " << robotPose.fi << endl;
    for(int k = 0; k < laserMeasurement->numberOfScans; k++)
    {
        double distance = laserMeasurement->Data[k].scanDistance;
        double xp = - (robotPose.x + distance * 2 * sin((360.0 - laserMeasurement->Data[k].scanAngle) * DEG2RAD));
        double yp = robotPose.y + distance * 2 * cos((360.0 - laserMeasurement->Data[k].scanAngle) * DEG2RAD);
        double fp = laserMeasurement->Data[k].scanAngle * DEG2RAD;

        MapPoint p = tfRealToMap({xp, yp, fp});

        setPointValue(p, getPointValue(p) + (unsigned short) 2);
//        cout << p.x << ", " << p.y << endl;

    }}

MapPoint RobotMap::getSize() {
    return {data.rows, data.cols};
}

cv::Mat RobotMap::getCVMatMap() {
    // TODO filter probability
    //filterSpeckles()

    // we have to clone cv::Mat, instead on we return only reference to cv::Mat
    return data.clone();
}

int RobotMap::getResolution() {
    return resolution;
}

void RobotMap::setPointValue(MapPoint point, unsigned short value) {
    if (!containPoint(point)) {
        throw invalid_argument("RobotMap: Set point is out of range ( " + to_string(point.x) + ", " + to_string(point.y) + "), size: ("
        + to_string(getSize().x) + ", " + to_string(getSize().y) + ")");
    }
    data.at<ushort>(Point(point.x, point.y)) = (value > UINT16_MAX) ? UINT16_MAX : value;
}

unsigned short RobotMap::getPointValue(MapPoint point) {
    if (!containPoint(point)) {
        throw invalid_argument("RobotMap: Get point is out of range ( " + to_string(point.x) + ", " + to_string(point.y) + "), size: ("
                               + to_string(getSize().x) + ", " + to_string(getSize().y) + ")");
    }
    return data.at<ushort>(Point(point.x, point.y));
}

RobotMap RobotMap::filterSpeckles() {
    //TODO filtrovanie neziaducich flakov na zaklade pravdepodobnosti
    // pouzi nasledujucu funkciu, je TOP
    // threshold( data, output, threshold_value, max_output_value, cv::THRESH_BINARY );

    return RobotMap("");
}

RobotMap RobotMap::getRobotMap() {
    // TODO filter map probability, return map only as 0, 1
    // filterSpeckles()

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
//    cv::waitKey(0);
}

MapPoint RobotMap::tfRealToMap(RobotPose realSpacePose) {
    MapPoint mapPose;
    mapPose.x = int(round(realSpacePose.x / resolution) + data.rows / 2);
    mapPose.y = int(data.cols / 2 - round(realSpacePose.y / resolution));
    if (__glibc_unlikely(!containPoint(mapPose))) {
        throw invalid_argument("RobotMap: tfRealToMap: transformation result is out of map range ( " + to_string(realSpacePose.x) + ", " + to_string(realSpacePose.y) + "), size: ("
                               + to_string(getSize().x) + ", " + to_string(getSize().y) + ")");
    }
    return mapPose;
}

RobotPose RobotMap::tfMapToReal(MapPoint mapSpacePose) {
    RobotPose realSpacePose;
    realSpacePose.x = (mapSpacePose.x - data.rows / 2) * resolution;
    realSpacePose.y = (data.cols / 2 - mapSpacePose.y) * resolution;
    realSpacePose.fi = 0;
    return realSpacePose;
}
