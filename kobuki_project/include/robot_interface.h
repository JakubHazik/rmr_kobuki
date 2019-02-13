//
// Created by jakub on 13.2.2019.
//

#ifndef KOBUKI_PROJECT_ROBOT_INTERFACE_H
#define KOBUKI_PROJECT_ROBOT_INTERFACE_H

#include <vector>
#include <cmath>

#include<iostream>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include <thread>
#include <mutex>


#define IP_ADDRESS "192.168.1.12"


typedef struct {

    unsigned short x;
    unsigned short y;
    unsigned short z;

} TRawGyroData;

typedef struct {
    //Hardware Version
    unsigned char HardwareVersionMajor;
    unsigned char HardwareVersionMinor;
    unsigned char HardwareVersionPatch;
    //Firmware Version
    unsigned char FirmwareVersionMajor;
    unsigned char FirmwareVersionMinor;
    unsigned char FirmwareVersionPatch;

    //Unique Device IDentifier(UDID)
    unsigned int UDID0;
    unsigned int UDID1;
    unsigned int UDID2;
    //Controller Info
    unsigned char PIDtype;
    unsigned int PIDgainP;
    unsigned int PIDgainI;
    unsigned int PIDgainD;
} TExtraRequestData;

typedef struct {
    //---zakladny balik
    unsigned short timestamp;
    //narazniky
    bool BumperLeft;
    bool BumperCenter;
    bool BumperRight;
    //cliff
    bool CliffLeft;
    bool CliffCenter;
    bool CliffRight;
    // padnutie kolies
    bool WheelDropLeft;
    bool WheelDropRight;
    //tocenie kolies
    unsigned short EncoderRight;
    unsigned short EncoderLeft;
    unsigned char PWMright;
    unsigned char PWMleft;
    //gombiky
    unsigned char ButtonPress;// 0 nie, 1 2 4 pre button 0 1 2 (7 je ze vsetky tri)
    //napajanie
    unsigned char Charger;
    unsigned char Battery;
    unsigned char overCurrent;
    //---docking ir
    unsigned char IRSensorRight;
    unsigned char IRSensorCenter;
    unsigned char IRSensorLeft;
    //---Inertial Sensor Data
    signed short GyroAngle;
    unsigned short GyroAngleRate;
    //---Cliff Sensor Data
    unsigned short CliffSensorRight;
    unsigned short CliffSensorCenter;
    unsigned short CliffSensorLeft;
    //---Current
    unsigned char wheelCurrentLeft;
    unsigned char wheelCurrentRight;
    //---Raw Data Of 3D Gyro
    unsigned char frameId;
    std::vector<TRawGyroData> gyroData;
    //---General Purpose Input
    unsigned short digitalInput;
    unsigned short analogInputCh0;
    unsigned short analogInputCh1;
    unsigned short analogInputCh2;
    unsigned short analogInputCh3;
    //---struktura s datami ktore sa nam tam objavia iba na poziadanie
    TExtraRequestData extraInfo;
} TKobukiData;

//
//class CKobuki {
//public:
//    CKobuki() = default;;
//
//    virtual ~CKobuki() = default;;
//
//
//
////    std::vector<unsigned char> setDefaultPID();
//};

typedef struct {
    int scanQuality;
    double scanAngle;
    double scanDistance;
} LaserData;

typedef struct {
    int numberOfScans;
    LaserData Data[1000];
} LaserMeasurement;


class RobotInterface {
    RobotInterface();
    
    void sendTranslationSpeed(double mmPerSec);

    void sendRotationSpeed(double radPerSec);

    void sendArcSpeed(double mmPerSec, double radius);

    LaserMeasurement getLaserData();

private:
    TKobukiData robotdata;
    LaserMeasurement laserData;
    std::mutex laserDataMutex;

    const std::string ipaddress = IP_ADDRESS;

    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other, las_si_posli;

    int las_s, las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other, rob_si_posli;

    int rob_s, rob_recv_len;
    unsigned int rob_slen;


    void t_processRobotData();

    void t_processLaserData();

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
