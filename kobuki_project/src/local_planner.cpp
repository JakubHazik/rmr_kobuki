//
// Created by jakub on 31.3.2019.
//

#include <include/local_planner.h>

enum MOVEMENT_STATES {
    READ_GLOBAL_POINT,
    CHECK_GLOBAL_FIRST_POINT,
    COMPUTE_BYPASS,
    WAIT_FOR_GLOBAL_POINT,
    WAIT_FOR_BYPASS_POINT
};


LocalPlanner::LocalPlanner(RobotInterface *robotInterface, LidarInterface *lidarInterface) {
    this->robotInterface = robotInterface;
    this->lidarInterface = lidarInterface;
}

void LocalPlanner::processMovement(list<RobotPose> globalWaypoints) {
    if (globalWaypoints.empty()) {
        syslog(LOG_WARNING, "[Local planner]: Local plan is without waypoints");
        return;
    }

    std::future<void> goalAchieved_fut;
    std::future<void> zoneAchieved_fut;
    RobotPose goalPoint;
    list<RobotPose> bypassWaypoints;
    MOVEMENT_STATES state = READ_GLOBAL_POINT;

    this->waypoints = globalWaypoints;

    while (true) {
        /* 1 zoberiem prvy bod z G listu
           2 ak je bod v kolizii tak ho daj prec a chod na 1
           3 ak je cesta v kolizii tak naplanuch obchadzku
                - ak dosiahnem zonu obchadzky tak preplanuj obdzadzku na novo
         */

        RobotMap localMap = lidarInterface->getRobotMap();

        stringstream ss;
        ss<<"Global: "<<waypoints.size()<<", Bypass: "<<bypassWaypoints.size()<<endl;
        syslog(LOG_INFO, "%s", ss.str().c_str());
        printRobotPose(goalPoint);


        switch (state) {
            // pop goal point
            case READ_GLOBAL_POINT: {
                syslog(LOG_NOTICE, "[Local planner]: STATE READ_GLOBAL_POINT");

                goalPoint = waypoints.front();
                state = CHECK_GLOBAL_FIRST_POINT;
                break;
            }

            // check goal point
            case CHECK_GLOBAL_FIRST_POINT: {
                syslog(LOG_NOTICE, "[Local planner]: STATE CHECK_GLOBAL_FIRST_POINT");

                if (pointCollisionCheck(localMap, goalPoint)) {
                    // point is in collision
                    syslog(LOG_NOTICE, "[Local planner]: Goal point in collision, remove it");
                    waypoints.pop_front();
                    if (waypoints.empty()) {
                        syslog(LOG_NOTICE, "[Local planner]: Waypoints empty");
                        return;
                    }
                    state = READ_GLOBAL_POINT;
                    continue;
                }

                if (pathCollisionCheck(localMap, goalPoint)) {
                    // path is in collision
                    syslog(LOG_NOTICE, "[Local planner]: Goal path in collision, make bypass");
                    state = COMPUTE_BYPASS;
                    continue;
                }

                goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
                zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
                state = WAIT_FOR_GLOBAL_POINT;
                break;
            }

            // compute bypass
            case COMPUTE_BYPASS: {
                syslog(LOG_NOTICE, "[Local planner]: STATE COMPUTE_BYPASS");

                bypassWaypoints = computeBypass(goalPoint);
                if (bypassWaypoints.empty()) {
                    state = READ_GLOBAL_POINT;
                    continue;
                }

                goalPoint = bypassWaypoints.front();
                goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
                zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);

                if (zoneAchieved_fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    // if first bypass waypoint zone is already achieved, go to the next
                    bypassWaypoints.pop_front();

                    if (!bypassWaypoints.empty()) {
                        goalPoint = bypassWaypoints.front();
                        goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
                        zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
                    }
                }

                state = WAIT_FOR_BYPASS_POINT;
                break;
            }

            // wait for G point
            case WAIT_FOR_GLOBAL_POINT: {
                syslog(LOG_NOTICE, "[Local planner]: STATE WAIT_FOR_GLOBAL_POINT");

                zoneAchieved_fut.wait();
                if (!waypoints.empty()) {
                    waypoints.pop_front();
                }

                if (waypoints.empty()) {
                    syslog(LOG_NOTICE, "[Local planner]: Wait for achieve last goal");
                    goalAchieved_fut.wait();
                    syslog(LOG_NOTICE, "[Local planner]: Movement has been processed");
                    return;
                }
                state = READ_GLOBAL_POINT;
                break;
            }

            // wait for B point
            case WAIT_FOR_BYPASS_POINT: {
                syslog(LOG_NOTICE, "[Local planner]: STATE WAIT_FOR_BYPASS_POINT");

                zoneAchieved_fut.wait();
                if (!bypassWaypoints.empty()) {
                    bypassWaypoints.pop_front();
                }

//                if (!bypassWaypoints.empty()) {
//                    goalPoint = bypassWaypoints.front();
//                    goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
//                    zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
//                    zoneAchieved_fut.wait();
//                }

                state = READ_GLOBAL_POINT;
                break;
            }

            default:break;
        }
    }


    syslog(LOG_NOTICE, "[Local planner]: Wait for achieve last goal");
    goalAchieved_fut.wait();
    syslog(LOG_NOTICE, "[Local planner]: Movement has been processed");
}

list<RobotPose> LocalPlanner::computeBypass(RobotPose goalPoint) {

    RobotMap localMap = lidarInterface->getRobotMap();

    try {
        GlobalPlanner globalPlanner(localMap, robotInterface->getOdomData(), goalPoint, Kconfig::HW::ROBOT_WIDTH);
        images_mtx.lock();
        floodFillImage = globalPlanner.getFloodFillImage();
        waypointsImage = globalPlanner.getWayPointsImage();
        pathImage = globalPlanner.getPathImage();
        images_mtx.unlock();

        auto bypassPoints = globalPlanner.getRobotWayPoints();
        bypassPoints.pop_back();    // last point is useless, I keep it in this->waypoints
        syslog(LOG_INFO, "[Local planner]: Bypass successful planned");
        return bypassPoints;

    } catch (NoPathException &e) {
        syslog(LOG_WARNING, "[Local planner]: Not able to plan bypass, try remove scan map");
//        lidarInterface->clearMap();

//        try {
//            GlobalPlanner globalPlanner(localMap, robotInterface->getOdomData(), goalPoint, Kconfig::HW::ROBOT_WIDTH);
//            images_mtx.lock();
//            floodFillImage = globalPlanner.getFloodFillImage();
//            waypointsImage = globalPlanner.getWayPointsImage();
//            pathImage = globalPlanner.getPathImage();
//            images_mtx.unlock();
//
//            auto bypassPoints = globalPlanner.getRobotWayPoints();
//            bypassPoints.pop_back();    // last point is useless, I keep it in this->waypoints
//            syslog(LOG_INFO, "[Local planner]: Bypass successful planned");
//            return bypassPoints;
//
//        } catch (NoPathException &e) {
//            syslog(LOG_WARNING, "[Local planner]: Really not able to plan bypass");
//
//            floodFillImage = cv::Mat();
//            waypointsImage = cv::Mat();
//            pathImage = cv::Mat();
//
//            return list<RobotPose>();
//        }

        floodFillImage = cv::Mat();
        waypointsImage = cv::Mat();
        pathImage = cv::Mat();

        return list<RobotPose>();
    }
}

bool LocalPlanner::pathCollisionCheck(RobotMap& localMap, RobotPose goalWaypoint) {
    auto mapSize = localMap.getSize();
    auto mapResolution = localMap.getResolution();
    MapPoint robotPoint = RobotMap::tfRealToMap(robotInterface->getOdomData(), mapSize, mapResolution);
    MapPoint goalPoint = RobotMap::tfRealToMap(goalWaypoint, mapSize, mapResolution);

    cv::Mat trajectoryLine = cv::Mat::zeros(cv::Size(mapSize.x, mapSize.y), CV_16UC1);
    cv::line(trajectoryLine,
             cv::Point(robotPoint.x, robotPoint.y),
             cv::Point(goalPoint.x, goalPoint.y),
             cv::Scalar(1),
             Kconfig::HW::ROBOT_WIDTH/mapResolution + 1);

    cv::Mat envMap = localMap.getCVMatMap();

    cv::Mat c = envMap*30000 + trajectoryLine*30000;
    cv::imshow("collision path", c);

    cv::Mat collision;
    cv::bitwise_and(envMap, trajectoryLine, collision);

    return cv::countNonZero(collision) != 0;
}

bool LocalPlanner::pointCollisionCheck(RobotMap& localMap, RobotPose goalWaypoint) {
    auto mapSize = localMap.getSize();
    auto mapResolution = localMap.getResolution();
    MapPoint goalPoint = RobotMap::tfRealToMap(goalWaypoint, mapSize, mapResolution);

    cv::Mat trajectoryLine = cv::Mat::zeros(cv::Size(mapSize.x, mapSize.y), CV_16UC1);
    cv::line(trajectoryLine,
             cv::Point(goalPoint.x, goalPoint.y),
             cv::Point(goalPoint.x, goalPoint.y),
             cv::Scalar(1),
             Kconfig::HW::ROBOT_WIDTH/mapResolution);

    cv::Mat envMap = localMap.getCVMatMap();

    cv::Mat c = envMap*30000 + trajectoryLine*30000;
    cv::imshow("collision point", c);
    cv::Mat collision;
    cv::bitwise_and(envMap, trajectoryLine, collision);

    return cv::countNonZero(collision) != 0;
}


void LocalPlanner::stopMovement() {
    // todo fix
    waypoints_mtx.lock();
    waypoints.clear();
    waypoints_mtx.unlock();
    robotInterface->setRequiredPoseOffset(robotInterface->getOdomData(), SPACE::ORIGIN_SPACE);
}

cv::Mat LocalPlanner::getPathImage() {
    lock_guard<mutex> lk(images_mtx);
    return pathImage;
}

cv::Mat LocalPlanner::getWayPointsImage() {
    lock_guard<mutex> lk(images_mtx);
    return waypointsImage;
}

cv::Mat LocalPlanner::getFloodFillImage() {
    lock_guard<mutex> lk(images_mtx);
    return floodFillImage;
}

void LocalPlanner::addWallBoundaries(RobotMap& map) {

    int boundariesSize = static_cast<int>(ceil(Kconfig::HW::ROBOT_WIDTH / 2 / map.getResolution()));

    cv::Mat referenceMap = map.getCVMatMap();
    cv::Mat tmpMap = referenceMap.clone();
    cv::Mat resultMap = referenceMap.clone();

    for (int b = 0; b < boundariesSize; b++){
        for (int i = 1; i < (int) directions.size(); i++) {
            tmpMap = translateMap(referenceMap, directions[i]);
            cv::bitwise_or(resultMap, tmpMap, resultMap);
        }
        referenceMap = resultMap.clone();
    }

    map = RobotMap(resultMap, map.getResolution());
}

cv::Mat LocalPlanner::translateMap(const cv::Mat &map, MapDirection direction) {
    cv::Mat output;
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, direction.x, 0, 1, direction.y);    // create transformation matrix for translate map
    cv::warpAffine(map, output, trans_mat, map.size());     // translate image
    return output;
}