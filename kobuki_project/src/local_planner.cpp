//
// Created by jakub on 31.3.2019.
//

#include <include/local_planner.h>


LocalPlanner::LocalPlanner(RobotInterface *robotInterface, LidarInterface *lidarInterface, list<RobotPose> globalWaypoints) {
    this->robotInterface = robotInterface;
    this->lidarInterface = lidarInterface;
    this->waypoints = globalWaypoints;

    robotInterface->setZoneParams(GOAL_ZONE_DISTANCE, boost::bind(&LocalPlanner::wayPointZoneAchieved_cbk, this));
}


//void LocalPlanner::goToGoal(RobotPose goalPose) {
//    robotInterface->setRequiredPose(goalPose);
//
//    // block function until goal will be achieved
//    std::unique_lock<std::mutex> lk(cv_mutex);
//    goalAchieved.wait(lk);
//
//    // TODO treba zisti ci je to vlastne v roznych vlaknach
//    std::thread::id this_id = std::this_thread::get_id();
//    cout<<"goToGoal "<<this_id<<endl;
//}

void LocalPlanner::wayPointZoneAchieved_cbk() {
    /*
     * funkcia je zavolana ked robot dosiahne zonu
     *  - ak este existuju waypointy tak urob pop na dosiahnuty a nastav dalsi v poradi
     */

    std::thread::id this_id = std::this_thread::get_id();
    cout<<"wayPointZoneAchieved_clb "<<this_id<<endl;

    waypoints_mtx.lock();

    if (!waypoints.empty()) {
        waypoints.pop_front();
    }

    if (!waypoints.empty()) {
        robotInterface->setRequiredPose(waypoints.front());
    }
    waypoints_mtx.unlock();
}

void LocalPlanner::processMovement() {
    waypoints_mtx.lock();
    if (waypoints.empty()) {
        return;
    }
    robotInterface->setRequiredPose(waypoints.front());
    waypoints_mtx.unlock();

    while (true) {
        waypoints_mtx.lock();
        if (waypoints.empty()) {
            break;
        }
        waypoints_mtx.unlock();

        if (computeBypass().empty()) {
            usleep(1000);
            continue;
        }


//        std::unique_lock<std::mutex> lk(cv_mutex);
//        goalAchieved.wait(lk);

    }

    while (!robotInterface->isGoalAchieved()) {
        usleep(1000);
    }

    cout<<"L planner hotovo";
}

list<RobotPose> LocalPlanner::computeBypass() {
    // TODO implementovat na lokalnej mape z lidar interface

    return list<RobotPose>();
}

