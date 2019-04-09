//
// Created by martin on 21.3.2019.
//

#ifndef KOBUKI_PROJECT_KOBUKI_H
#define KOBUKI_PROJECT_KOBUKI_H

#include <include/own_typedefs.h>
#include <include/robot_interface.h>
#include <include/lidar_interface.h>
#include <include/robot_map.h>
#include <include/local_planner.h>
#include <include/global_planner.h>
#include <include/visualizer.h>

#include <list>

#define MAP_SIZE RobotPose{6000 * 2, 6000 * 2, 0}   // dvojnasobny rozmer mapy
#define MAP_RESOLUTION 50                           // mm per pixel
#define ROBOT_WIDTH 360                             // mm

using namespace std;

class Kobuki {
public:
    explicit Kobuki();

    virtual ~Kobuki();

    cv::Mat getEnvironmentAsImage(void *argumentyNejake); // toto bude vraciat veci z visualizera, navolim si ze co vsetko chcem mat v image

    void loadMapFromFile(string filepath); //argument mozno bude aj velkost mapy

    void setRobotActualPosition(double x, double y, double fi);     //ked sa nacita nejaka mapa tak treba nastavit robota do takej pozicie aby bol spravne usadeny v mape

    void sendRobotToPosition(double x, double y);        //nastavenie ciela robota, tu sa spustia plannery

    void updateGlobalMap();  /// todo toto je riadna ******

    //TODO toto predsa nemoze byt public
    RobotInterface *robotInterface;
    LidarInterface *lidarInterface;
    RobotMap map;
private:

};

#endif //KOBUKI_PROJECT_KOBUKI_H