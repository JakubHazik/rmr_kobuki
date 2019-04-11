//
// Created by martin on 21.3.2019.
//

#include <include/kobuki.h>

using namespace std;

Kobuki::Kobuki() : map(Kconfig::Defaults::MAP_SIZE, Kconfig::Defaults::MAP_RESOLUTION) {
    robotInterface = new RobotInterface();
    lidarInterface = new LidarInterface(Kconfig::Defaults::MAP_SIZE, Kconfig::Defaults::MAP_RESOLUTION, robotInterface);
    lPlanner = new LocalPlanner(robotInterface, lidarInterface);
}

Kobuki::~Kobuki() {
    delete robotInterface;
    delete lidarInterface;
}


//void Kobuki::updateGlobalMap(){
//    LaserMeasurement laserData = lidarInterface->getLaserData();
//    RobotPose odometry = robotInterface->getOdomData();
//
//    syslog(LOG_DEBUG, "Update global map");
//    map.addMeasurement(odometry, &laserData);
//}

void Kobuki::moveRobotToPosition(double x, double y, SPACE space) {
    RobotPose goalPose = {x, y, 0};
    RobotPose odom = robotInterface->getOdomData();

    if (space == SPACE::ROBOT_SPACE) {
        goalPose = RobotInterface::robot2originSpace(odom, goalPose);
    }

    GlobalPlanner gPlanner(map, odom, goalPose, Kconfig::HW::ROBOT_WIDTH);
    list<RobotPose> waypoints = gPlanner.getRobotWayPoints();

    gPlannerFloodFill = gPlanner.getFloodFillImage();
    gPlannerPath = gPlanner.getPathImage();
    gPlannerWaypoints = gPlanner.getWayPointsImage();

//    waypoints = {{1000,0}, {1000,1000}, {0,0,0}};

//    list<RobotPose> waypoints = {goalPose};

    lPlanner->processMovement(waypoints);
}

void Kobuki::setRobotActualPosition(double x, double y, double fi) {
    robotInterface->resetOdom(x, y, fi);
    robotInterface->setRequiredPose({});    // also set goal to 0,0, because robot will want to move
}

cv::Mat Kobuki::getEnvironmentAsImage(PLANNER_TYPE planner, bool environment, bool waypoints, bool path, bool floodFill,
                                      bool laserScan) {
    Visualizer visualizer(map.getSize(), Kconfig::HW::ROBOT_WIDTH, map.getResolution());

    if (environment) {
        visualizer.environmentMap = map.getCVMatMap();
    }

    if (laserScan) {
        visualizer.laserScan = lidarInterface->getRobotMap().getCVMatMap();
    }

    if (planner == PLANNER_TYPE::GLOBAL) {
        if (waypoints) {
            visualizer.waypoints = gPlannerWaypoints;
        }

        if (path) {
            visualizer.path = gPlannerPath;
        }

        if (floodFill) {
            visualizer.floodFill = gPlannerFloodFill;
        }
    } else if (planner == PLANNER_TYPE::LOCAL) {
        if (waypoints) {
            visualizer.waypoints = lPlanner->getWayPointsImage();
        }

        if (path) {
            visualizer.path = lPlanner->getPathImage();
        }

        if (floodFill) {
            visualizer.floodFill = lPlanner->getFloodFillImage();
        }
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

void Kobuki::clearMap() {
    map.clearMap();
}

RobotPose Kobuki::getRobotPosition() {
    return robotInterface->getOdomData();
}

void Kobuki::stopRobotMovement() {
    lPlanner->stopMovement();
}
