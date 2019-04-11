//
// Created by jakub on 28.3.2019.
//

#include <include/visualizer.h>

using namespace cv;

Visualizer::Visualizer(MapSize mapSize, int robotWidth, int mapResolution) {
    this->mapSize = mapSize;
    this->robotWidth = robotWidth;
    this->mapResolution = mapResolution;
}

cv::Mat Visualizer::getImage(RobotPose robotPose) {
    Mat output = cv::Mat(mapSize.x, mapSize.y, CV_8UC3);
    output.setTo(Kconfig::Visualizer::COLOR_BACKGROUND);         // set image to white color

    if (!environmentMap.empty()) {
        addImageMask(output, environmentMap, Kconfig::Visualizer::COLOR_BLACK);
    }

    if (!floodFill.empty()) {
        addFloodFill(output, floodFill);
    }

    if (!path.empty()) {
        addImageMask(output, path, Kconfig::Visualizer::COLOR_PATH);
    }

    if (!waypoints.empty()) {
        addImageMask(output, waypoints, Kconfig::Visualizer::COLOR_WAYPOINTS);
    }

    if (!laserScan.empty()) {
        addImageMask(output, laserScan, Kconfig::Visualizer::COLOR_LASER);
    }

    addRobot(output, robotPose, Kconfig::Visualizer::COLOR_ROBOT);

    return output;
}


void Visualizer::addRobot(cv::Mat &inputOutput, RobotPose robotPose, cv::Scalar color) {
    double fi = robotPose.fi;
    int x = (int)round((cos(fi)) * robotWidth/2/mapResolution);
    int y = (int)round((sin(fi)) * robotWidth/2/mapResolution);

    MapPoint robot = RobotMap::tfRealToMap(robotPose, mapSize, mapResolution);
    circle(inputOutput, Point(robot.x, robot.y), robotWidth/2/mapResolution, color);
    arrowedLine(inputOutput, Point(robot.x, robot.y), Point(robot.x + x, robot.y + y), color);
}


void Visualizer::addImageMask(cv::Mat &output, cv::Mat &addImg, cv::Scalar color) {
    addImg.convertTo(addImg, CV_8UC3);
    cvtColor(addImg, addImg, COLOR_GRAY2RGB);
    output.setTo(color, addImg);
}

void Visualizer::addFloodFill(Mat &output, Mat &addImg) {
    Mat mask, mask_inv;
    addImg.convertTo(addImg, CV_8UC3);
    threshold(addImg, mask, 1, 255, cv::THRESH_BINARY);
    bitwise_not(mask, mask_inv);

    Mat output_bg, addImg_bg;
    cvtColor(addImg, addImg, COLOR_GRAY2RGB);
    bitwise_and(output, output, output_bg, mask_inv);
    bitwise_and(addImg, addImg, addImg_bg, mask);

    Mat color = Mat(addImg_bg.size(), CV_8UC3, Kconfig::Visualizer::COLOR_BLACK);
    color.setTo(Kconfig::Visualizer::COLOR_FLOOD_FILL, mask);

    addImg_bg += color;
    output = output_bg + addImg_bg;
}





