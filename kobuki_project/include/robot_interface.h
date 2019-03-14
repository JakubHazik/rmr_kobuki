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

#define ROBOT_IP_ADDRESS    "192.168.1.12"
#define ROBOT_TICK_TO_METER 0.085292090497737556558
#define ROBOT_TICK_TO_RAD   0.002436916871363930187454
#define ROBOT_ENCODER_MAX   0xFFFF  // max of unsigned short
#define ROBOT_GYRO_MAX      0x7FFF  // TODO toto je zle

#define ROBOT_WHEEL_RADIUS  35      // [mm]
#define ROBOT_WHEEL_BASE    230     // [mm]
#define ROBOT_THRESHOLD_RADIUS_GYRO_COMPUTATION 100 // [mm]

#define ROBOT_MAX_SPEED_FORWARD 300 // [mm / s]
#define ROBOT_MIN_SPEED_FORWARD 30  // [mm / s]
#define ROBOT_ACCELERATION      40  // [mm / s^-2]
#define ROBOT_REG_P 0.7


#define ROBOT_REG_ACCURACY      30  // [mm] accuracy of positioning

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

#define ROBOT_POSE_CONTROLLER_PERIOD 0.1 // [s]
#define ROBOT_ARC_MOVE_RADIUS_LIMIT 32000



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


/**
 * Nech je pozicia x,y v jednotkach [mm]
 */
typedef struct {
    double x;
    double y;
    double fi;
} RobotPose;

class RobotInterface {
public:
    RobotInterface();

    ~RobotInterface();

    enum RobotStates {READ_POINT, STOP, MOVE, ANOTHER_CONTROL};
    RobotStates actualRobotState = READ_POINT;

//    /**
//     * Go to requested position
//     * @param position RobotPose object - position of x[mm], y[mm] and fi[rad]
//     * @param leadingEdge - to create leading Edge (ramp start)
//     * @param trailingEdge - to create trailing Edge (ramp stop)
//     */
//    void goToPosition(RobotPose position, bool leadingEdge, bool trailingEdge);

    void addCommandToQueue(const RobotPose &cmd);
    
    void sendTranslationSpeed(int mmPerSec);

    void sendRotationSpeed(int radPerSec);

    void sendArcSpeed(int mmPerSec, int mmRadius);

    void resetOdom();

    RobotPose getOdomData();

    bool sendDataToRobot(std::vector<unsigned char> mess);

    /**
     * Util function sends created data to robot via socket
     * @param data Data to be sent
     * @return Success of sending
     */
//    bool sendDataToRobot(std::vector<unsigned char> data);

    bool forOdomUseGyro =false;

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

    std::queue<RobotPose> robotCmdPoints;
    std::mutex robotCmdPoints_mtx;

    /*
     * ========================================
     * Funkcie
     */
    void t_readRobotData();

    void computeOdometry(unsigned short encoderRight, unsigned short encoderLeft, signed short gyroAngle);

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

//    /**
//     * PID regulator
//     * @param w requested value
//     * @param y current value
//     * @param saturation saturation of maximal output
//     * @return
//     */
//    double wheelPID(double error, double saturation);

    double getAbsoluteDistance(RobotPose posA, RobotPose posB);

    double getRotationError(RobotPose robotPose, RobotPose goalPose);

    /**
     * Angle fitting function to rotation radius
     * Constants calculated with MATLAB-s fitting toolbox
     *
     * Source: $PROJECT_ROOT/research/angle_regulator_function.m
     *
     * @param angle Angle [rad] to fit to radius
     * @return rotation radius according to angle
     */
    int fitRotationRadius(double rotationError);

    void t_poseController();

    void setRobotStatus(RobotStates newStatus);

    int speedRegulator(double error);

    int speedRadiusCorrection(int requiredSpeed, int radius);
};

#endif //KOBUKI_PROJECT_ROBOT_INTERFACE_H
