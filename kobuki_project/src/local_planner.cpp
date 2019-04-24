//
// Created by jakub on 31.3.2019.
//

#include <include/local_planner.h>

//enum MOVEMENT_STATES {
//    READ_G_POINT,
//    CHECK_G_FIRST_POINT
//};


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
    int state = 0;

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
            case 0: {
                syslog(LOG_NOTICE, "[Local planner]: STATE 0");

                goalPoint = waypoints.front();
                state = 1;
                break;
            }

            // check goal point
            case 1: {
                syslog(LOG_NOTICE, "[Local planner]: STATE 1");

                if (pointCollisionCheck(localMap, goalPoint)) {
                    // point is in collision
                    syslog(LOG_NOTICE, "[Local planner]: Goal point in collision, remove it");
                    waypoints.pop_front();
                    if (waypoints.empty()) {
                        syslog(LOG_NOTICE, "[Local planner]: Waypoints empty");
                        return;
                    }
                    state = 0;
                    continue;
                }

                if (pathCollisionCheck(localMap, goalPoint)) {
                    // path is in collision
                    syslog(LOG_NOTICE, "[Local planner]: Goal path in collision, make bypass");
                    state = 2;
                    continue;
                }

                goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
                zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
                state = 3;
                break;
            }

            // compute bypass
            case 2: {
                syslog(LOG_NOTICE, "[Local planner]: STATE 2");

                bypassWaypoints = computeBypass(goalPoint);
                if (bypassWaypoints.empty()) {
                    state = 0;
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

                state = 4;
                break;
            }

            // wait for G point
            case 3: {
                syslog(LOG_NOTICE, "[Local planner]: STATE 3");

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
                state = 0;
                break;
            }

            // wait for B point
            case 4: {
                syslog(LOG_NOTICE, "[Local planner]: STATE 4");

                zoneAchieved_fut.wait();
                bypassWaypoints.pop_front();

//                if (!bypassWaypoints.empty()) {
//                    goalPoint = bypassWaypoints.front();
//                    goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
//                    zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
//                    zoneAchieved_fut.wait();
//                }

                state = 0;
                break;
            }

            default:break;
        }
    }


    syslog(LOG_NOTICE, "[Local planner]: Wait for achieve last goal");
    goalAchieved_fut.wait();
    syslog(LOG_NOTICE, "[Local planner]: Movement has been processed");
//
//
//
//    waypoints_mtx.lock();
//    this->waypoints = globalWaypoints;
//    waypoints_mtx.unlock();
//
//    while (true) {
//        waypoints_mtx.lock();
//        if (waypoints.empty()) {
//            waypoints_mtx.unlock();
//            break;
//        }
//        waypoints_mtx.unlock();
//
//        // go to next point if zone is achieved
//        if (zoneAchieved_fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
//            // check which goal point has been achieved
//            if (!bypassWaypoints.empty()) {
//                // bypass waypoint has been achieved
//                bypassWaypoints.pop_front();
//            } else {
//                // global waypoint has been achieved
//                waypoints_mtx.lock();
//                waypoints.pop_front();
//                waypoints_mtx.unlock();
//            }
//
//            // decide from which list get the next point
//            if (!bypassWaypoints.empty()) {
//                // if we have a bypass waypoint go there
//                goalPoint = bypassWaypoints.front();
//
//            } else {
//                // if we have a global waypoint go there
//                waypoints_mtx.lock();
//
//                if (waypoints.empty()) {
//                    // bypass and global waypoints are empty, so we can brake loop
//                    waypoints_mtx.unlock();
//                    break;
//                }
//
//                goalPoint = waypoints.front();
//                waypoints_mtx.unlock();
//            }
//
//            goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
//            zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
//        }
//
//        // zone is not achieved yet, check collisions
//        bypassWaypoints = computeBypass(goalPoint);
//        if (!bypassWaypoints.empty()) {
//            goalPoint = bypassWaypoints.front();
//            goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
//            zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
//        }
//
////        // if collision algorithm create bypass waypoints, add them to the list (front)
////        if (!bypass_waypoints.empty()) {
////            // add bypass_waypoints to waypoints list
////            waypoints_mtx.lock();
////            waypoints.insert(waypoints.begin(), bypass_waypoints.begin(), bypass_waypoints.end());
////            waypoints_mtx.unlock();
////            goalAchieved_fut = robotInterface->setRequiredPose(bypass_waypoints.front());
////            zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
////        }
//    }

//    syslog(LOG_NOTICE, "[Local planner]: Wait for achieve last goal");
//    goalAchieved_fut.wait();
//    syslog(LOG_NOTICE, "[Local planner]: Movement has been processed");
}

list<RobotPose> LocalPlanner::computeBypass(RobotPose goalPoint) {
//    waypoints_mtx.lock();
//    auto firstWaypoint = waypoints.front();
//    waypoints_mtx.unlock();

    RobotMap localMap = lidarInterface->getRobotMap();
//
//    if (!pathCollisionCheck(localMap, goalPoint)) {
//        // collision has not been found
//        return list<RobotPose>();
//    }
//    syslog(LOG_INFO, "[Local planner]: Path collision detected");

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
        syslog(LOG_WARNING, "[Local planner]: Not able to plan bypass");

        floodFillImage = cv::Mat();
        waypointsImage = cv::Mat();
        pathImage = cv::Mat();

        std::lock_guard<mutex> lk(waypoints_mtx);

        //waypoints.pop_front();

//        RobotPose bypass
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
             Kconfig::HW::ROBOT_WIDTH/mapResolution);

//    cv::imshow("trajectory", trajectoryLine);
//    cv::waitKey(0);
//    addWallBoundaries(localMap);
    cv::Mat envMap = localMap.getCVMatMap();

//    //test
//    robotPoint = RobotMap::tfRealToMap({100,500}, mapSize, mapResolution);
//    goalPoint = RobotMap::tfRealToMap({-100, -500}, mapSize, mapResolution);
//    cv::line(envMap,
//             cv::Point(robotPoint.x, robotPoint.y),
//             cv::Point(goalPoint.x, goalPoint.y),
//             cv::Scalar(1),
//             Kconfig::HW::ROBOT_WIDTH/mapResolution);

//    cv::imshow("laser", envMap);
//    cv::waitKey(0);
//
    cv::Mat c = envMap*30000 + trajectoryLine*30000;
    cv::imshow("collision", c);

    cv::Mat collision;
    cv::bitwise_and(envMap, trajectoryLine, collision);
//    cv::imshow("bit", collision);
//    cv::waitKey(0);

    return cv::countNonZero(collision) != 0;
}

bool LocalPlanner::pointCollisionCheck(RobotMap& localMap, RobotPose goalWaypoint) {
    auto mapSize = localMap.getSize();
    auto mapResolution = localMap.getResolution();
//    MapPoint robotPoint = RobotMap::tfRealToMap(robotInterface->getOdomData(), mapSize, mapResolution);
    MapPoint goalPoint = RobotMap::tfRealToMap(goalWaypoint, mapSize, mapResolution);

    cv::Mat trajectoryLine = cv::Mat::zeros(cv::Size(mapSize.x, mapSize.y), CV_16UC1);
    cv::line(trajectoryLine,
             cv::Point(goalPoint.x, goalPoint.y),
             cv::Point(goalPoint.x, goalPoint.y),
             cv::Scalar(1),
             Kconfig::HW::ROBOT_WIDTH/mapResolution);

//    cv::imshow("point", trajectoryLine * 30000);
//    cv::waitKey(0);

//    addWallBoundaries(localMap);
    cv::Mat envMap = localMap.getCVMatMap();

//    //test
//    robotPoint = RobotMap::tfRealToMap({100,500}, mapSize, mapResolution);
//    goalPoint = RobotMap::tfRealToMap({-100, -500}, mapSize, mapResolution);
//    cv::line(envMap,
//             cv::Point(robotPoint.x, robotPoint.y),
//             cv::Point(goalPoint.x, goalPoint.y),
//             cv::Scalar(1),
//             Kconfig::HW::ROBOT_WIDTH/mapResolution);

//    cv::imshow("s", envMap);
//    cv::waitKey(0);
//
//    cv::Mat collision = envMap + trajectoryLine;
//
//    cv::imshow("s", collision);
//    cv::waitKey(0);

    cv::Mat collision;
    cv::bitwise_and(envMap, trajectoryLine, collision);
//    cv::imshow("collision", collision*30000);
//    cv::waitKey(0);

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