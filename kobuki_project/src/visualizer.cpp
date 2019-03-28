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
    output.setTo(COLOR_WHITE);         // set image to white color

    if (!environmentMap.empty()) {
        addImageMask(output, environmentMap, COLOR_BLACK);
    }

    if (!path.empty()) {
        addImageMask(output, path, COLOR_GREEN);
    }

    if (!waypoints.empty()) {
        addImageMask(output, waypoints, COLOR_BLUE);
    }

    addRobot(output, robotPose, COLOR_RED);

    return output;
}


void Visualizer::addRobot(cv::Mat &inputOutput, RobotPose robotPose, cv::Scalar color) {

    double fi = robotPose.fi;
    int x = (int)round((cos(fi*DEG2RAD)) * robotWidth/mapResolution);
    int y = (int)round((sin(fi*DEG2RAD)) * robotWidth/mapResolution);

    MapPoint robot = RobotMap::tfRealToMap(robotPose, mapSize, mapResolution);
    circle(inputOutput, Point(robot.x, robot.y), robotWidth/mapResolution, color);
    arrowedLine(inputOutput, Point(robot.x, robot.y), Point(robot.x + x, robot.y + y), color);

}


void Visualizer::addImageMask(cv::Mat &output, cv::Mat &addImg, cv::Scalar color) {
    addImg.convertTo(addImg, CV_8UC3);
    cvtColor(addImg, addImg, COLOR_GRAY2RGB);
    output.setTo(color, addImg);
}

void Visualizer::addFloodFill(Mat &output, Mat &addImg) {
    Mat mask, mask_inv,img1Gray;

    cvtColor(addImg, img1Gray, COLOR_RGB2GRAY);
    threshold(img1Gray, mask, 1, 255, cv::THRESH_BINARY);
    bitwise_not(mask, mask_inv);

//    cv::imshow("a", output);
//    cv::waitKey(0);
//    cv::imshow("a", addImg);
//    cv::waitKey(0);
//
//    cv::imshow("mask", mask);
//    cv::waitKey(0);
//    cv::imshow("mask_inv", mask_inv);
//    cv::waitKey(0);


    Mat output_bg, addImg_bg;

    bitwise_and(output, output, output_bg, mask_inv);
    bitwise_and(addImg, addImg, addImg_bg, mask);

//    cv::imshow("output_bg", output_bg);
//    cv::waitKey(0);
//    cv::imshow("addImg_bg", addImg_bg);
//    cv::waitKey(0);

    output = output_bg + addImg_bg;

//    cv::imshow("result", output);
//    cv::waitKey(0);


//    retur?n Mat();
}





