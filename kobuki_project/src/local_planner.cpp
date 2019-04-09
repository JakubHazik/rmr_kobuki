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
            }
        }
    }

    goalAchieved_fut.wait();
    cout<<"L planner hotovo";
}

list<RobotPose> LocalPlanner::computeBypass() {
    // TODO implementovat na lokalnej mape z lidar interface

    return list<RobotPose>();
}

