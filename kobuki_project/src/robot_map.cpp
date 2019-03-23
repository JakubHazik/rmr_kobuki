//
// Created by jakub on 14.3.2019.
//

#include "include/robot_map.h"

using namespace std;
using namespace cv;

RobotMap::RobotMap(RobotPose mapSize, int resolution): resolution(resolution) {

    data = Mat::zeros(int(mapSize.x / resolution), int(mapSize.y / resolution), CV_16UC1);  // construct matrix with unsigned short values
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

cv::Mat RobotMap::getCVMatMap(int threshold_value) {
    /// Filter proability
    filterSpeckles(threshold_value);

    // we have to clone cv::Mat, instead on we return only reference to cv::Mat
    return outputMap.clone();
}

int RobotMap::getResolution() {
    return resolution;
}

void RobotMap::setPointValue(MapPoint point, unsigned short value) {
    if (__glibc_unlikely(!containPoint(point))) {
        throw invalid_argument("RobotMap: Set point is out of range ( " + to_string(point.x) + ", " + to_string(point.y) + "), size: ("
        + to_string(getSize().x) + ", " + to_string(getSize().y) + ")");
    }
    data.at<ushort>(Point(point.x, point.y)) = (value > USHRT_MAX) ? (unsigned short) USHRT_MAX : value;
}

unsigned short RobotMap::getPointValue(MapPoint point) {
    if (__glibc_unlikely(!containPoint(point))) {
        throw invalid_argument("RobotMap: Get point is out of range ( " + to_string(point.x) + ", " + to_string(point.y) + "), size: ("
                               + to_string(getSize().x) + ", " + to_string(getSize().y) + ")");
    }
    return data.at<ushort>(Point(point.x, point.y));
}

void RobotMap::filterSpeckles(int threshold_value) {
    /// @1 - input image
    /// @2 - output image
    /// @3 - threshold value
    /// @4 - binary MAX (all values above threshold)
    /// @5 - mode (type) of threshold
    threshold( data, outputMap, threshold_value, 1, cv::THRESH_BINARY );
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
    resize(output, output, Size(), SHOW_IMAGE_SCALE_FACTOR, SHOW_IMAGE_SCALE_FACTOR);
    imshow("Robot map", output);
    cv::waitKey(0);
}

MapPoint RobotMap::tfRealToMap(RobotPose realSpacePose) {
    MapPoint mapPose;
    mapPose.x = int(round(realSpacePose.x / resolution) + data.rows / 2);
    mapPose.y = int(data.cols / 2 - round(realSpacePose.y / resolution));
    if (__glibc_unlikely(!containPoint(mapPose))) {
        throw invalid_argument("RobotMap: tfRealToMap: transformation result is out of map range ( " + to_string(mapPose.x) + ", " + to_string(mapPose.y) + "), size: ("
                               + to_string(data.rows) + ", " + to_string(data.cols) + ")");
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

void RobotMap::loadIdealMap(std::string filename, RobotPose robotReference) {
    ifstream file;
    file.open(filename);

    string line;
    int pointsNum;
    char delimiter;
    double pointX;
    double pointY;
    vector<MapPoint> corners;

    while (!file.eof()) {
        file>>pointsNum;

        for(int i = 0; i < pointsNum; i++) {
            file>>delimiter;    // [
            file>>pointX;       // number
            file>>delimiter;    // ,
            file>>pointY;       // number
            file>>delimiter;    // ]

            corners.push_back(tfRealToMap(RobotPose{pointX * 10, pointY * 10}));    // also convert from [cm] to [mm]
        }
        corners.push_back(corners[0]);
        printWallToMap(corners);
        corners.clear();
    }

    // translate map center to robot reference
    auto translation = tfRealToMap(robotReference);
    translateMap({data.rows/2 - translation.x, data.cols/2 - translation.y});

    // rotate map around center to robot reference
    cv::Mat rotation = cv::getRotationMatrix2D(cv::Point(data.rows/2, data.cols/2), robotReference.fi, 1);
    warpAffine(data, data, rotation, data.size());

    //setPointValue(tfRealToMap({0,0,0}), 255);  // write center to the map
}

void RobotMap::printWallToMap(const std::vector<MapPoint> &corners) {
    for (int i = 0; i < (int) corners.size() - 1; i++) {
        for (int x = corners[i].x; x != corners[i + 1].x; (x < corners[i + 1].x)? x++: x--) {
            setPointValue({x, corners[i].y}, 1);
        }

        for (int y = corners[i].y; y != corners[i + 1].y; (y < corners[i + 1].y)? y++: y--) {
            setPointValue({corners[i].x, y}, 1);
        }
    }
}

void RobotMap::translateMap(MapPoint direction) {
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, direction.x, 0, 1, direction.y);    // create transformation matrix for translate map
    cv::warpAffine(data, data, trans_mat, data.size());     // translate image
}
