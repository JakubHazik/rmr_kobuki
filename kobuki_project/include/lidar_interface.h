//
// Created by martin on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_LIDAR_INTERFACE_H
#define KOBUKI_PROJECT_LIDAR_INTERFACE_H

#pragma once

#include <mutex>
#include <thread>
#include <syslog.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <include/own_typedefs.h>

#include <include/robot_interface.h>


using namespace std;

class LidarInterface {
public:
    LidarInterface();

    virtual ~LidarInterface();

    /**
     * Getter for laser data
     * @return <LaserMeasurement> structure
     */
    LaserMeasurement getLaserData();

    // Public for GUI - drawing
    std::mutex laserData_mtx;

private:

    std::thread laser;
    LaserMeasurement laserData;

    RobotInterface robot;

    const std::string ipAddress = ROBOT_IP_ADDRESS;

    /// UDP settings
    struct sockaddr_in socket_me, socket_other, socket_send;
    int socket_FD;
    ssize_t received_length;
    unsigned int socket_FD_length;

    char buf[50000];

    void t_readLaserData();
};

#endif //KOBUKI_PROJECT_LIDAR_INTERFACE_H
