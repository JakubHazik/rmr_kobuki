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

    // set first waypoint
    goalAchieved_fut = robotInterface->setRequiredPose(globalWaypoints.front());
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

        if (zoneAchieved_fut.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
            // zone is achieved
            waypoints_mtx.lock();
            waypoints.pop_front();
            if (!waypoints.empty()) {
                goalAchieved_fut = robotInterface->setRequiredPose(waypoints.front());
                zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);
            }
            waypoints_mtx.unlock();
        }

        // zone is not achieved yet
        // check collisions
        list<RobotPose> bypass_waypoints = computeBypass();

        // if collision algorithm create bypass waypoints, add them to the list (front)
        if (!bypass_waypoints.empty()) {
            // add bypass_waypoints to waypoints list
            waypoints_mtx.lock();
            waypoints.insert(waypoints.begin(), bypass_waypoints.begin(), bypass_waypoints.end());
            waypoints_mtx.unlock();
            goalAchieved_fut = robotInterface->setRequiredPose(bypass_waypoints.front());
            zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::PoseControl::GOAL_ZONE_DISTANCE);

        }
    }

    goalAchieved_fut.wait();
    syslog(LOG_NOTICE, "[Local planner]: Movement has been processed");
}

list<RobotPose> LocalPlanner::computeBypass() {
    waypoints_mtx.lock();
    auto firstWaypoint = waypoints.front();
    waypoints_mtx.unlock();

    RobotMap localMap = lidarInterface->getRobotMap();

    if (!collisionCheck(localMap, firstWaypoint)) {
        // collision has not been found
        return list<RobotPose>();
    }
    syslog(LOG_NOTICE, "[Local planner]: Collision detected");

    GlobalPlanner globalPlanner(localMap, robotInterface->getOdomData(), firstWaypoint, Kconfig::HW::ROBOT_WIDTH);
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
    waypoints_mtx.lock();
    waypoints.clear();
    waypoints_mtx.unlock();
    robotInterface->setRequiredPoseOffset(robotInterface->getOdomData(), SPACE::ORIGIN_SPACE);
}

