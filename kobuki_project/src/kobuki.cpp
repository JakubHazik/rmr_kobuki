//
// Created by martin on 21.3.2019.
//

#include <include/kobuki.h>

using namespace std;

Kobuki::Kobuki() : map(Kconfig::Defaults::MAP_SIZE, Kconfig::Defaults::MAP_RESOLUTION) {
    robotInterface = new RobotInterface();
    lidarInterface = new LidarInterface(Kconfig::Defaults::MAP_SIZE, Kconfig::Defaults::MAP_RESOLUTION, robotInterface);
}

Kobuki::~Kobuki() {
    delete robotInterface;
    delete lidarInterface;
}


void Kobuki::updateGlobalMap(){
    LaserMeasurement laserData = lidarInterface->getLaserData();
    RobotPose odometry = robotInterface->getOdomData();

    syslog(LOG_DEBUG, "Update global map");
    map.addMeasurement(odometry, &laserData);
}

void Kobuki::sendRobotToPosition(double x, double y) {
    GlobalPlanner gPlanner(map, robotInterface->getOdomData(), RobotPose{x, y, 0}, Kconfig::HW::ROBOT_WIDTH);
    list<RobotPose> waypoints = gPlanner.getRobotWayPoints();

    gPlannerFloodFill = gPlanner.getFloodFillImage();
    gPlannerPath = gPlanner.getPathImage();
    gPlannerWaypoints = gPlanner.getWayPointsImage();

//    waypoints = {{1000,0}, {1000,1000}, {0,0,0}};
    LocalPlanner lPlanner(robotInterface, lidarInterface, waypoints);
    lPlanner.processMovement();
}

void Kobuki::setRobotActualPosition(double x, double y, double fi) {
    robotInterface->resetOdom(x, y, fi);
}

cv::Mat Kobuki::getEnvironmentAsImage(bool environment, bool waypoints, bool path, bool floodFill, bool laserScan) {
    Visualizer visualizer(map.getSize(), Kconfig::HW::ROBOT_WIDTH, map.getResolution());

    if (environment) {
        visualizer.environmentMap = map.getCVMatMap();
    }

    if (waypoints) {
        visualizer.waypoints = gPlannerWaypoints;
    }

    if (path) {
        visualizer.path = gPlannerPath;
    }

    if (floodFill) {
        visualizer.floodFill = gPlannerFloodFill;
    }

    if (laserScan) {
        visualizer.laserScan = lidarInterface->getRobotMap().getCVMatMap();
    }

    return visualizer.getImage(robotInterface->getOdomData());
}

void Kobuki::loadMapFromFile(string filepath) {
    string extension = filepath.substr(filepath.find_last_of('.') + 1);

    if (extension == Kconfig::Defaults::IDEAL_MAP_EXTENSION) {
        map = RobotMap(filepath, true);
    } else if (extension == Kconfig::Defaults::OPENCV_MAP_EXTENSION) {
        map = RobotMap(filepath, false);
    } else {
        throw std::invalid_argument("File: " + filepath + " does not match with default extensions");
    }
}

void Kobuki::saveMapToFile(string filepath) {
    map.saveToFile(std::move(filepath));
}
