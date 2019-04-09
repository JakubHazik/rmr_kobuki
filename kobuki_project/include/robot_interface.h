//
// Created by jakub on 13.2.2019.
//

#ifndef KOBUKI_PROJECT_ROBOT_INTERFACE_H
#define KOBUKI_PROJECT_ROBOT_INTERFACE_H

#include <vector>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <math.h>
#include <queue>
#include "regulator.h"
#include "own_typedefs.h"
#include <include/robot_map.h>
#include <boost/signals2.hpp>
#include <future>

#define ROBOT_IP_ADDRESS    "192.168.1.12"
#define ROBOT_TICK_TO_METER 0.085292090497737556558
#define ROBOT_TICK_TO_RAD   0.002436916871363930187454
#define ROBOT_ENCODER_MAX   0xFFFF  // max of unsigned short
#define ROBOT_WHEEL_RADIUS  35      // [mm]
#define ROBOT_WHEEL_BASE    230     // [mm]
#define ROBOT_GOAL_ACCURACY      30  // [mm] accuracy of positioning
#define ROBOT_POSE_CONTROLLER_PERIOD 0.1 // [s]


class RobotInterface {
public:
    explicit RobotInterface();

    ~RobotInterface();

    std::future<void> setRequiredPose(RobotPose goalPose);

    std::future<void> setZoneParams(int goalZone);
    
    void sendTranslationSpeed(int mmPerSec);

    void sendRotationSpeed(int radPerSec);

    void sendArcSpeed(int mmPerSec, int mmRadius);

    void resetOdom(double x = 0, double y = 0, double fi = 0);

    RobotPose getOdomData();

    bool sendDataToRobot(std::vector<unsigned char> mess);

    bool isGoalAchieved();

private:
    /*
    * ========================================
    * Premenne
    */

    std::thread robot;

    TKobukiData robotData;

    RobotPose odom;
    std::mutex odom_mtx;

    const std::string ipAddress = ROBOT_IP_ADDRESS;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other, rob_si_posli;
    int rob_s, rob_recv_len;
    unsigned int rob_slen;

    Regulator poseRegulator;
    RobotPose goalPose = {0};
    std::mutex goalPose_mtx;
    std::promise<void> goalAchieved;

    std::mutex zone_mtx;
    int goalZone = 0;
    std::promise<void> zoneAchieved;

    /*
     * ========================================
     * Funkcie
     */
    void t_readRobotData();

    void computeOdometry(unsigned short encoderRight, unsigned short encoderLeft, signed short gyroAngle);

    double getAbsoluteDistance(RobotPose posA, RobotPose posB);

    void t_poseController();

    /*
     * Communication interface
     */

    std::vector<unsigned char> setTranslationSpeed(int mmpersec);

    std::vector<unsigned char> setRotationSpeed(double radpersec);

    std::vector<unsigned char> setArcSpeed(int mmpersec, int radius);

    std::vector<unsigned char> setSound(int noteinHz, int duration);

    std::vector<unsigned char> setDefaultPID();

    int checkChecksum(unsigned char *data);

    int parseKobukiMessage(TKobukiData &output, unsigned char *data);

    int fillData(TKobukiData &output, unsigned char *message) {
        return parseKobukiMessage(output, message);
    }
};

#endif //KOBUKI_PROJECT_ROBOT_INTERFACE_H
