//
// Created by jakub on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_REGULATOR_H
#define KOBUKI_PROJECT_REGULATOR_H

#include <math.h>
#include "own_typedefs.h"

#define ROBOT_MAX_SPEED_FORWARD 330 // [mm / s]
#define ROBOT_MIN_SPEED_FORWARD 30  // [mm / s]
#define ROBOT_ACCELERATION      50  // [mm / s^-2]
#define ROBOT_REG_P 0.7
#define ROBOT_REG_RADIUS_MIN 5
//#define ROBOT_ARC_MOVE_RADIUS_LIMIT 32000

class Regulator{
public:
    Regulator(double regPeriod);

    RegulatorAction getAction(const RobotPose &robotPose, const RobotPose &goalPose);
private:
    double regPeriod;
    int speedRadiusCorrection(int requiredSpeed, int radius);
    int speedRegulator(double error);
    int fitRotationRadius(double rotationError);
    double getRotationError(const RobotPose &robotPose, const RobotPose &goalPose);
    double getAbsoluteDistance(const RobotPose &poseA, const RobotPose &poseB);
};


#endif //KOBUKI_PROJECT_REGULATOR_H
