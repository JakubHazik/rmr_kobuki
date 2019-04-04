//
// Created by martin on 21.3.2019.
//

#ifndef KOBUKI_PROJECT_KOBUKI_H
#define KOBUKI_PROJECT_KOBUKI_H

#include <include/own_typedefs.h>

#include <include/robot_interface.h>
#include <include/lidar_interface.h>
#include <include/robot_map.h>
#include <include/global_planner.h>
#include <include/visualizer.h>

#define MAP_SIZE RobotPose{6000 * 2, 6000 * 2, 0}    // dvojnasobny rozmer mapy
#define MAP_RESOLUTION 50                            // mm per pixel

class Kobuki {
public:
    explicit Kobuki();

    virtual ~Kobuki();

    cv::Mat getEnvironmentAsImage(void *argumentyNejake); // toto bude vraciat veci z visualizera, navolim si ze co vsetko chcem mat v image

    void loadMapFromFile();

    void setRobotActualPosition(double x, double y, double fi);

    void updateGlobalMap();  /// todo toto je riadna ******



    //TODO toto predsa nemoze byt public
    RobotInterface *robotInterface;
    LidarInterface *lidarInterface;
    RobotMap map;
private:

};

#endif //KOBUKI_PROJECT_KOBUKI_H