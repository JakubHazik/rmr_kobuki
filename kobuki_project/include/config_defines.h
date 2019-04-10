//
// Created by jakub on 9.4.2019.
//

#ifndef KOBUKI_PROJECT_CONFIG_DEFINES_H
#define KOBUKI_PROJECT_CONFIG_DEFINES_H

#include "own_typedefs.h"
#include <opencv2/opencv.hpp>

/*
 * This is Kobuki Robot config file with all configuration variables
 */
namespace Kconfig {

    namespace Defaults {
        // default algorithm params
        static const std::string ROBOT_IP_ADDRESS = "192.168.1.12";
        static const RobotPose MAP_SIZE = RobotPose{6000 * 2, 6000 * 2, 0};         // dvojnasobny rozmer mapy
        static const int MAP_RESOLUTION = 50;                                       // mm per pixel
        static const std::string IDEAL_MAP_EXTENSION = "txt";
        static const std::string OPENCV_MAP_EXTENSION = "yaml";
    }

    namespace Ui {
        static const int CANVAS_REFRESH_RATE = 250;
    }

    namespace LidarControl {
        static const int DATA_HOLD_COEFFICIENT = 100;                               // number of scans which will be keep in local RobotMap
    }

    namespace PoseControl {
        static const int GOAL_ZONE_DISTANCE = 300;                                  // mm polomer kruhu okolo goal pozicie
        static const int GOAL_ACCURACY = 30;                                        // [mm] accuracy of positioning
        static const float POSE_CONTROLLER_PERIOD = 0.1;                            // [s]
    }

    namespace HW {
        // hardware params
        static const double TICK_TO_METER = 0.085292090497737556558;
        static const double TICK_TO_RAD = 0.002436916871363930187454;
        static const unsigned short ENCODER_MAX = 0xFFFF;                           // max of unsigned short
        static const int WHEEL_RADIUS = 35;                                         // [mm]
        static const int WHEEL_BASE = 230;                                          // [mm]
        static const int ROBOT_WIDTH = 360;                                         // mm
    }

    namespace Regulator {
        // regulator params
        static const int MAX_SPEED = 330;                                           // [mm / s]
        static const int MIN_SPEED = 30;                                            // [mm / s]
        static const int ACCELERATION = 50;                                         // [mm / s^-2]
        static const float PROPORTIONAL_PARAM = 0.7;
        static const int MIN_RADIUS = 5;
    }

    namespace Visualizer {
        // visualizer colors
        // COLOR VALUES Scalar(Blue, Green, Red)
        static const cv::Scalar COLOR_BACKGROUND = cv::Scalar(255, 255, 255);        //white
        static const cv::Scalar COLOR_ROBOT = cv::Scalar(0, 0, 255);                 //red
        static const cv::Scalar COLOR_PATH = cv::Scalar(0, 255, 0);                  //green
        static const cv::Scalar COLOR_WAYPOINTS = cv::Scalar(255, 0, 0);             //blue
        static const cv::Scalar COLOR_LASER = cv::Scalar(15, 40, 160);               //wine
        static const cv::Scalar COLOR_FLOOD_FILL = cv::Scalar(20, 140, 100);         //yellow
        static const cv::Scalar COLOR_BLACK = cv::Scalar(1, 1, 1);
    }
}

#endif //KOBUKI_PROJECT_CONFIG_DEFINES_H
