//
// Created by jakub on 14.3.2019.
//

#include "include/regulator.h"

// vracia znamienko, -1, 0, 1
template <typename T> int signum(T val) {
    return (T(0) < val) - (val < T(0));
}

Regulator::Regulator(double regPeriod): regPeriod(regPeriod) {

}

double Regulator::getAbsoluteDistance(const RobotPose &poseA, const RobotPose &poseB) {
    return sqrt(pow(poseA.x - poseB.x, 2) + pow(poseA.y - poseB.y, 2));
}

int Regulator::speedRadiusCorrection(int requiredSpeed, int radius) {

    float result = round(requiredSpeed + requiredSpeed * 0.05 - requiredSpeed/((fabs(radius) + 500.0)/500.0));

    if (result == 0) {
        result = 1;
    }

    // todo znamienka + saturacia
    return int(result);
}

int Regulator::speedRegulator(double error) {
    static int lastOutput = 0;

    double output = error * ROBOT_REG_P;

    // saturation
    if (output > ROBOT_MAX_SPEED_FORWARD) {
        output = ROBOT_MAX_SPEED_FORWARD;

        // ked plati tato podmienka tak robot chce zrychlovat, urobme to po rampe
        if (lastOutput < output) {
            output = lastOutput + ROBOT_ACCELERATION * regPeriod;
            if (output < ROBOT_MIN_SPEED_FORWARD) {
                output = ROBOT_MIN_SPEED_FORWARD;
            }
        }
    }

//    output = output * pow(error/ROBOT_MAX_SPEED_FORWARD,0.3);

    lastOutput = int(output);
    return int(output);
}

int Regulator::fitRotationRadius(double rotationError) {

    static const double coef_a = -1.0432e+03;
    static const double coef_b = -2.9754;
    static const double coef_c = -2.3957e+04;
    static const double coef_d = -28.0845;

    double radius;

    // toto je kvoli tomu pretoze signum(0) = 0, a to je blbe
    if (rotationError == 0) {
        radius = coef_a * exp(coef_b * fabs(rotationError)) + coef_c * exp(coef_d * fabs(rotationError)) -1 - ROBOT_REG_RADIUS_MIN;
    } else {
        radius = -1 * signum(rotationError) * (coef_a * exp(coef_b * fabs(rotationError)) + coef_c * exp(coef_d * fabs(rotationError)) -1 -ROBOT_REG_RADIUS_MIN);
    }

    return int(round(radius));
}


double Regulator::getRotationError(const RobotPose &robotPose, const RobotPose &goalPose) {
    double goalAngle = atan2(goalPose.y - robotPose.y, goalPose.x - robotPose.x);
    //syslog(LOG_NOTICE, "atan2: Y: %f; X: %f; %f[%f]", goalPose.y - robotPose.y, goalPose.x - robotPose.x, goalAngle*RAD2DEG,goalAngle);

    double robotAngle = robotPose.fi;

    double result;

    if (signum(robotAngle) == signum(goalAngle)) {
        // uhly patria do jednej polroviny
        return goalAngle - robotAngle;
    }

    if (robotAngle > 0) {
        result = -1 * (fabs(goalAngle) + fabs(robotAngle));
    } else {
        result = fabs(goalAngle) + fabs(robotAngle);
    }

    if (result > M_PI) {
        result = -2*M_PI + fabs(robotAngle) + fabs(goalAngle);
    }

    if (result < -M_PI) {
        result = 2*M_PI - fabs(robotAngle) - fabs(goalAngle);
    }

    return result;
}

regulatorAction Regulator::getAction(const RobotPose &robotPose, const RobotPose &goalPose) {
    double translationError = getAbsoluteDistance(robotPose,  goalPose);
    double rotationError = getRotationError(robotPose, goalPose);

    int actionRadius = fitRotationRadius(rotationError);
    int actionSpeed = speedRegulator(translationError);
    actionSpeed = speedRadiusCorrection(actionSpeed, actionRadius);

    regulatorAction action;
    action.radius = actionRadius;
    action.speed = actionSpeed;

//    syslog(LOG_DEBUG, "Remaining distance: %.lf; Calculated speed: %d, %d; Rotation error: %f[%f]; Calculated radius: %d",
//           translationError, speed, speed_correction, rotationError*RAD2DEG, rotationError, radius);
    return action;
}
