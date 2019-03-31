//
// Created by jakub on 14.3.2019.
//

#include "include/global_planner.h"


bool operator==(const MapPoint &p1, const MapPoint &p2)
{
    return p1.x == p2.x && p1.y == p2.y;
}

MapPoint operator+(const MapPoint &p1, const MapPoint &p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
}

GlobalPlanner::GlobalPlanner(RobotMap map, RobotPose startPose, RobotPose goalPose, int robotWidth):
    map(map),
    floodMap(cv::Mat::zeros(map.getSize().x, map.getSize().y, CV_16UC1), map.getResolution()),
    pathMap(cv::Mat::zeros(map.getSize().x, map.getSize().y, CV_8UC1), map.getResolution()),
    wayPoints(cv::Mat::zeros(map.getSize().x, map.getSize().y, CV_8UC1), map.getResolution()),
    robotWidth(robotWidth)
    {

    addWallBoundaries();
    setStartEndPose(startPose, goalPose);
    floodFill();

    try{
        findPathPoints();
    } catch (NoPathException &e) {
        syslog(LOG_ERR, "%s", e.what());
        robotWayPoints = list<RobotPose>();     // clear queue
    }
}

void GlobalPlanner::setStartEndPose(RobotPose startPose, RobotPose goalPose) {
    // save start pose to class member
    startPoint = map.tfRealToMap(startPose);

    if (map.getPointValue(startPoint) == 1) {
        string message = "GlobalPLanner: start pose is in wall or to close to wall";
        syslog(LOG_ERR, "%s", message.c_str());
        throw invalid_argument(message);
    }

    // save goal pose map as number 2
    goalPoint = map.tfRealToMap(goalPose);

    if (map.getPointValue(goalPoint) == 1) {
        string message = "GlobalPLanner: goal pose is in wall or to close to wall";
        syslog(LOG_ERR, "%s", message.c_str());
        throw invalid_argument(message);
    }
}

void GlobalPlanner::addWallBoundaries() {
    int boundariesSize = static_cast<int>(ceil(robotWidth / 2 / this->map.getResolution()));

    cv::Mat referenceMap = this->map.getCVMatMap();
    cv::Mat tmpMap = referenceMap.clone();
    cv::Mat resultMap = referenceMap.clone();

    for (int b = 0; b < boundariesSize; b++){
        for (int i = 1; i < (int) directions.size(); i++) {
            tmpMap = translateMap(referenceMap, directions[i]);
            cv::bitwise_or(resultMap, tmpMap, resultMap);
        }
        referenceMap = resultMap.clone();
    }

    this->map = RobotMap(resultMap, this->map.getResolution());
}

void GlobalPlanner::floodFill() {
    queue<FloodPoint> floodPoints;

    // add first (goal) point, it has value = 2
    floodPoints.push({goalPoint, 2});

    while (!floodPoints.empty()) {
        FloodPoint floodPoint = floodPoints.front();
        floodPoints.pop();

        if (!map.containPoint(floodPoint.mapPoint) || map.getPointValue(floodPoint.mapPoint) != 0) {
            continue;
        }

        map.setPointValue(floodPoint.mapPoint, floodPoint.color);

        if (__glibc_unlikely(floodPoint.mapPoint == startPoint)) {
            return;
        }

        floodPoints.push({{floodPoint.mapPoint.x + 1, floodPoint.mapPoint.y + 1}, (unsigned short)(floodPoint.color + 1)});
        floodPoints.push({{floodPoint.mapPoint.x + 1, floodPoint.mapPoint.y    }, (unsigned short)(floodPoint.color + 1)});
        floodPoints.push({{floodPoint.mapPoint.x + 1, floodPoint.mapPoint.y - 1}, (unsigned short)(floodPoint.color + 1)});
        floodPoints.push({{floodPoint.mapPoint.x    , floodPoint.mapPoint.y - 1}, (unsigned short)(floodPoint.color + 1)});
        floodPoints.push({{floodPoint.mapPoint.x - 1, floodPoint.mapPoint.y - 1}, (unsigned short)(floodPoint.color + 1)});
        floodPoints.push({{floodPoint.mapPoint.x - 1, floodPoint.mapPoint.y    }, (unsigned short)(floodPoint.color + 1)});
        floodPoints.push({{floodPoint.mapPoint.x - 1, floodPoint.mapPoint.y + 1}, (unsigned short)(floodPoint.color + 1)});
        floodPoints.push({{floodPoint.mapPoint.x    , floodPoint.mapPoint.y + 1}, (unsigned short)(floodPoint.color + 1)});
    }
}

void GlobalPlanner::findPathPoints() {
    FFDirection direction = NONE, directionOld = NONE;

    MapPoint actualPoint = startPoint;
    pathPoints.push_back(actualPoint);

    while (!(actualPoint == goalPoint)) {
        actualPoint = findNextDirection(actualPoint, direction);

        if (__glibc_unlikely(direction != directionOld)) {
            robotWayPoints.push_back(map.tfMapToReal(pathPoints.back()));
        }

        pathPoints.push_back(actualPoint);
        directionOld = direction;
    }

    if (__glibc_likely(!robotWayPoints.empty())) {
        robotWayPoints.pop_front();         // remove first (start) point
    }

    robotWayPoints.push_back(map.tfMapToReal(goalPoint));    // add last (goalPoint) to waypoints queue
}

MapPoint GlobalPlanner::findNextDirection(const MapPoint &point, FFDirection &direction) {
    const int pointValue = map.getPointValue(point);
    int nextPointValue;

    try {  // catch if point is outside of the map
        nextPointValue = map.getPointValue(point + directions[direction]);
    } catch (invalid_argument &e) {
        nextPointValue = 0;     // if value is 0, next condition step this point
    }

    // prefer previous direction
    if (__glibc_likely(pointValue > nextPointValue && nextPointValue > 1)) {
        return point + directions[direction];
    }

    for (int i = 1; i < (int) directions.size(); i++) {
        try { // catch if point is outside of the map
            nextPointValue = map.getPointValue(point + directions[i]);
        } catch (invalid_argument &e) {
            nextPointValue = 0; // if value is 0, next condition step this point
        }

        if (pointValue > nextPointValue && nextPointValue > 1) {
            direction = (FFDirection)i;
            return point + directions[i];
        }
    }

    throw NoPathException("GlobalPlanner: findNextDirection: floodfill have no fall gradient in point: [" + to_string(point.x) + ", " + to_string(point.y) + "]");
}

list<RobotPose> GlobalPlanner::getRobotWayPoints() {
    return robotWayPoints;
}

cv::Mat GlobalPlanner::translateMap(const cv::Mat &map, MapDirection direction) {
    cv::Mat output;
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, direction.x, 0, 1, direction.y);    // create transformation matrix for translate map
    cv::warpAffine(map, output, trans_mat, map.size());     // translate image
    return output;
}

cv::Mat GlobalPlanner::getPathImage() {
    for (const auto &point: pathPoints) {
        pathMap.setPointValue(point, 1);
    }

    return pathMap.getCVMatMap();
}

cv::Mat GlobalPlanner::getWayPointsImage() {
    for (const auto &point: robotWayPoints) {
        wayPoints.setPointValue(wayPoints.tfRealToMap(point), 1);
    }

    return wayPoints.getCVMatMap();
}

cv::Mat GlobalPlanner::getFloodFillImage() {
    cv::Mat mask;
    cv::Mat output;
    cv::Mat cv_map = map.getCVMatMap();

    cv::threshold(cv_map, mask, 2, USHRT_MAX, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8UC1);
    cv_map.copyTo(output, mask);

    return output;
}
