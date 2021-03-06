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
#include <numeric>
#include <future>
#include <algorithm>

#include "robot_interface.h"
#include "lidar.h"
#include "own_typedefs.h"
#include "config_defines.h"


using namespace std;

class LidarInterface {
public:
    LidarInterface(RobotPose mapSize, int mapResolution, RobotInterface *_robotInterface);

    virtual ~LidarInterface();

    /**
     * Getter for laser data
     * @return <LaserMeasurement> structure
     */
    LaserMeasurement getLaserData();

    // Public for GUI - drawing
    std::mutex laserData_mtx;

    RobotMap getRobotMap();     // vrati nejaku svoju mapu 'localMap'

    //void setMeasurementCallback(function<void(LaserMeasurement)> callback);
    void clearMap();

private:

    RobotInterface *robot;

    /// localMap contains (int) Kconfig::LidarControl::DATA_HOLD_COEFFICIENT of last scans
    /// Given coefficient presents maximum number in map as well, so measurement with this value
    /// is the newest one and every single lower value of point represents older measurement
    /// At the end, 0 value represents no obstacle or forgotten obstacle older than DATA_HOLD_COEFFICIENT cycles
    RobotMap localMap;
    std::mutex localMap_mtx;

    std::thread laser_thread;
    std::atomic_bool laserDataThreadRun = {true};
    LaserMeasurement laserData;

    Lidar lidar;

    /// UDP settings
    struct sockaddr_in socket_me, socket_other, socket_send;
    int socket_FD;
    ssize_t received_length;
    unsigned int socket_FD_length;

    char buf[50000];

    void t_readLaserData();

    /**
     * Local map update function, adds measurement to map with forgetting factor
     * @param laserData
     */
    void updateLocalMap(LaserMeasurement laserData);
};

#endif //KOBUKI_PROJECT_LIDAR_INTERFACE_H
