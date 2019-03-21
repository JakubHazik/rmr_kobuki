//
// Created by martin on 21.3.2019.
//

#ifndef KOBUKI_PROJECT_KOBUKI_H
#define KOBUKI_PROJECT_KOBUKI_H

#include <include/own_typedefs.h>

#include <include/robot_interface.h>
#include <include/lidar_interface.h>
#include <include/robot_map.h>



class Kobuki {
public:
    explicit Kobuki();

    virtual ~Kobuki();

    RobotInterface robotInterface;
    LidarInterface lidarInterface;
    RobotMap       mapInterface;

    void updateGlobalMap();

private:
    void startPeriodic(int seconds);




};

#endif //KOBUKI_PROJECT_KOBUKI_H