//
// Created by jakub on 31.3.2019.
//

#include <include/local_planner.h>


LocalPlanner::LocalPlanner(RobotInterface *robotInterface, LidarInterface *lidarInterface, list<RobotPose> globalWaypoints) {
    this->robotInterface = robotInterface;
    this->lidarInterface = lidarInterface;
    this->waypoints = globalWaypoints;
}

void LocalPlanner::processMovement() {
    if (waypoints.empty()) {
        return;
    }
    std::future<void> goalAchieved_fut;
    std::future<void> zoneAchieved_fut;

    goalAchieved_fut = robotInterface->setRequiredPose(waypoints.front());
    zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::Defaults::GOAL_ZONE_DISTANCE);

    while (true) {
        if (waypoints.empty()) {
            break;
        }

        auto fut_status = zoneAchieved_fut.wait_for(std::chrono::milliseconds(50));

        if (fut_status == std::future_status::ready) {
            // zone is achieved
            waypoints.pop_front();
            if (!waypoints.empty()) {
                goalAchieved_fut = robotInterface->setRequiredPose(waypoints.front());
                zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::Defaults::GOAL_ZONE_DISTANCE);
            }
        } else {
            // zone is not achieved yet
            // check collisions
            list<RobotPose> bypass_waypoints = computeBypass();

            // if collision algorithm create bypass waypoints, add them to the list (front)
            if (!bypass_waypoints.empty()) {
                // add bypass_waypoints to waypoints list
                waypoints.insert(waypoints.begin(), bypass_waypoints.begin(), bypass_waypoints.end());
                goalAchieved_fut = robotInterface->setRequiredPose(waypoints.front());
                zoneAchieved_fut = robotInterface->setZoneParams(Kconfig::Defaults::GOAL_ZONE_DISTANCE);
            }
        }
    }

    goalAchieved_fut.wait();
    cout<<"L planner hotovo";
}

list<RobotPose> LocalPlanner::computeBypass() {

    RobotMap localMap = lidarInterface->getRobotMap();

    if (!collisionCheck(localMap)) {
        // collision has not been found
        return list<RobotPose>();
    }

    GlobalPlanner globalPlanner(localMap, robotInterface->getOdomData(), waypoints.front(), Kconfig::HW::ROBOT_WIDTH);

    return globalPlanner.getRobotWayPoints();
}

bool LocalPlanner::collisionCheck(RobotMap &localMap) {
    
    auto mapSize = localMap.getSize();
    auto mapResolution = localMap.getResolution();
    MapPoint robotPoint = RobotMap::tfRealToMap(robotInterface->getOdomData(), mapSize, mapResolution);
    MapPoint goalPoint = RobotMap::tfRealToMap(waypoints.front(), mapSize, mapResolution);

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
    cv::imshow("bit", collision);
    cv::waitKey(0);

    return cv::countNonZero(collision) != 0;
}

