//
// Created by jakub on 14.3.2019.
//

#ifndef KOBUKI_PROJECT_REGULATOR_H
#define KOBUKI_PROJECT_REGULATOR_H

#include <math.h>
#include "own_typedefs.h"
#include "config_defines.h"

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
