//
// Created by martin on 21.3.2019.
//

#include <include/kobuki.h>

using namespace std;

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

void Kobuki::sendRobotToPosition(double x, double y) {
    GlobalPlanner gPlanner(map, robotInterface->getOdomData(), RobotPose{x, y, 0}, ROBOT_WIDTH);
    list<RobotPose> waypoints = gPlanner.getRobotWayPoints();

//    list<RobotPose> waypoints = {{0,0,0}, {1000,0}, {1000,1000}, {0,0,0}};
    LocalPlanner lPlanner(robotInterface, lidarInterface, waypoints);
    lPlanner.processMovement();
}

void Kobuki::setRobotActualPosition(double x, double y, double fi) {
    robotInterface->resetOdom(x, y, fi);
}
