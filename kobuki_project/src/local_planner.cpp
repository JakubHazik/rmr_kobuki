//
// Created by jakub on 31.3.2019.
//

#include <include/local_planner.h>


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
    RobotPose goalPoint = globalWaypoints.front();
    list<RobotPose> bypassWaypoints;

    // set first waypoint
    goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
    zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);

    waypoints_mtx.lock();
    this->waypoints = globalWaypoints;
    waypoints_mtx.unlock();

    while (true) {
        waypoints_mtx.lock();
        if (waypoints.empty()) {
            waypoints_mtx.unlock();
            break;
        }
        waypoints_mtx.unlock();

        // go to next point if zone is achieved
        if (zoneAchieved_fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            // check which goal point has been achieved
            if (!bypassWaypoints.empty()) {
                // bypass waypoint has been achieved
                bypassWaypoints.pop_front();
            } else {
                // global waypoint has been achieved
                waypoints_mtx.lock();
                waypoints.pop_front();
                waypoints_mtx.unlock();
            }

            // decide from which list get the next point
            if (!bypassWaypoints.empty()) {
                // if we have a bypass waypoint go there
                goalPoint = bypassWaypoints.front();

            } else {
                // if we have a global waypoint go there
                waypoints_mtx.lock();

                if (waypoints.empty()) {
                    // bypass and global waypoints are empty, so we can brake loop
                    waypoints_mtx.unlock();
                    break;
                }

                goalPoint = waypoints.front();
                waypoints_mtx.unlock();
            }

            goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
            zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
        }

        // zone is not achieved yet, check collisions
        bypassWaypoints = computeBypass(goalPoint);
        if (!bypassWaypoints.empty()) {
            goalPoint = bypassWaypoints.front();
            goalAchieved_fut = robotInterface->setRequiredPose(goalPoint);
            zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
        }

//        // if collision algorithm create bypass waypoints, add them to the list (front)
//        if (!bypass_waypoints.empty()) {
//            // add bypass_waypoints to waypoints list
//            waypoints_mtx.lock();
//            waypoints.insert(waypoints.begin(), bypass_waypoints.begin(), bypass_waypoints.end());
//            waypoints_mtx.unlock();
//            goalAchieved_fut = robotInterface->setRequiredPose(bypass_waypoints.front());
//            zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
//        }
    }

    goalAchieved_fut.wait();
    syslog(LOG_NOTICE, "[Local planner]: Movement has been processed");
}

list<RobotPose> LocalPlanner::computeBypass(RobotPose goalPoint) {
//    waypoints_mtx.lock();
//    auto firstWaypoint = waypoints.front();
//    waypoints_mtx.unlock();

    RobotMap localMap = lidarInterface->getRobotMap();

    if (!collisionCheck(localMap, goalPoint)) {
        // collision has not been found
        return list<RobotPose>();
    }
    syslog(LOG_NOTICE, "[Local planner]: Collision detected");

    GlobalPlanner globalPlanner(localMap, robotInterface->getOdomData(), goalPoint, Kconfig::HW::ROBOT_WIDTH);

    images_mtx.lock();
    floodFillImage = globalPlanner.getFloodFillImage();
    waypointsImage = globalPlanner.getWayPointsImage();
    pathImage = globalPlanner.getPathImage();
    images_mtx.unlock();

    auto bypassPoints = globalPlanner.getRobotWayPoints();
    bypassPoints.pop_back();    // last point is useless, I keep it in this->waypoints
    return bypassPoints;
}

bool LocalPlanner::collisionCheck(RobotMap &localMap, RobotPose goalWaypoint) {
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

//    cv::imshow("s", trajectoryLine);
//    cv::waitKey(0);

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
//    cv::imshow("bit", collision);
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

