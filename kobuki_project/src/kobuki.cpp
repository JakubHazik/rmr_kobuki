//
// Created by martin on 21.3.2019.
//

#include <include/kobuki.h>

// Kobuki::Kobuki() : mapInterface((MapSize){120,120}, 100) {
Kobuki::Kobuki() : mapInterface("../map_dataset.yaml") {

}

Kobuki::~Kobuki() {

}

void Kobuki::startPeriodic(int seconds){

}

void Kobuki::updateGlobalMap(){
    LaserMeasurement laserData = lidarInterface.getLaserData();
    RobotPose odometry = robotInterface.getOdomData();

    syslog(LOG_DEBUG, "Update global map");
    mapInterface.addMeasurement(odometry, &laserData);
}