//
// Created by martin on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_LIDAR_INTERFACE_H
#define KOBUKI_PROJECT_LIDAR_INTERFACE_H

#include <mutex>
#include <thread>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <stdlib.h>

#include <include/robot_interface.h>

using namespace std;

typedef struct {
    int scanQuality;
    double scanAngle;
    double scanDistance;
} LaserData;

typedef struct {
    int numberOfScans;
    LaserData Data[1000];
} LaserMeasurement;


class LidarInterface {
public:
    LidarInterface();

    ~LidarInterface();

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

    const std::string ipAddress = ROBOT_IP_ADDRESS;

    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other, las_si_posli;
    int las_s, las_recv_len;
    unsigned int las_slen;

    void t_readLaserData();


};

#endif //KOBUKI_PROJECT_LIDAR_INTERFACE_H
