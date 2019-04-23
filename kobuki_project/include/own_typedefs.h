//
// Created by jakub on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_OWN_TYPEDEFS_H
#define KOBUKI_PROJECT_OWN_TYPEDEFS_H

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

#include <vector>
#include <iostream>

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


typedef struct {
    int scanQuality;
    double scanAngle;
    double scanDistance;
} LaserData;

typedef struct {
    int numberOfScans;
    LaserData Data[1000];
} LaserMeasurement;

typedef struct {
    double x;   // mm
    double y;   // mm
    double fi;  // rad
} RobotPose;

typedef struct {
    int speed;
    int radius;
} RegulatorAction;

typedef struct {
    int x;
    int y;
} MapSize;

typedef MapSize MapPoint;

inline bool operator==(const RobotPose &p1, const RobotPose &p2)
{
    return p1.x == p2.x && p1.y == p2.y && p1.fi == p2.fi;
}

inline bool operator==(const MapPoint &p1, const MapPoint &p2)
{
    return p1.x == p2.x && p1.y == p2.y;
}

inline MapPoint operator+(const MapPoint &p1, const MapPoint &p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
}

enum SPACE {
    ROBOT_SPACE,
    ORIGIN_SPACE,
};

inline void printRobotPose(const RobotPose &p) {
    std::cout<<"X: "<<p.x<<", Y:" << p.y << ", Fi: " << p.fi<<std::endl;
}

#endif //KOBUKI_PROJECT_OWN_TYPEDEFS_H
