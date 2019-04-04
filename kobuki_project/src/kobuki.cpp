//
// Created by martin on 21.3.2019.
//

#include <include/kobuki.h>



Kobuki::Kobuki() : map(MAP_SIZE, MAP_RESOLUTION) {
    robotInterface = new RobotInterface();
    lidarInterface = new LidarInterface(MAP_SIZE, MAP_RESOLUTION);




}

Kobuki::~Kobuki() {
    delete robotInterface;
    delete lidarInterface;
    // pozor na to vlakna vo vnutri objektov stale bezia
}


void Kobuki::updateGlobalMap(){
    LaserMeasurement laserData = lidarInterface->getLaserData();
    RobotPose odometry = robotInterface->getOdomData();

//    syslog(LOG_DEBUG, "Update global map");
    map.addMeasurement(odometry, &laserData);
}