//
// Created by martin on 21.3.2019.
//

#include <include/kobuki.h>

 Kobuki::Kobuki() : map((RobotPose){12*1000,12*1000}, 30) {
//Kobuki::Kobuki() : map("../map_dataset.yaml") {

}

Kobuki::~Kobuki() {

}


void Kobuki::updateGlobalMap(){
    LaserMeasurement laserData = lidarInterface.getLaserData();
    RobotPose odometry = robotInterface.getOdomData();

//    syslog(LOG_DEBUG, "Update global map");
    map.addMeasurement(odometry, &laserData);
}