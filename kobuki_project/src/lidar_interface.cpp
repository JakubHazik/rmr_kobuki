//
// Created by martin on 14.3.2019.
//

#include <include/lidar_interface.h>

LidarInterface::LidarInterface(RobotPose mapSize, int mapResolution, RobotInterface *_robotInterface): localMap(mapSize, mapResolution) {
    laser_thread = thread(&LidarInterface::t_readLaserData, this);
    robot = _robotInterface;
}

LidarInterface::~LidarInterface() {
    laserDataThreadRun = false;
    laser_thread.join();
}

void LidarInterface::t_readLaserData() {
    syslog(LOG_INFO, "[LidarInterface]: ReadLaserData thread started");
    // Initialize Winsock

    socket_FD_length = sizeof(socket_other);
    if ((socket_FD = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        syslog(LOG_ERR, "[LidarInterface]: Create socket failed");
    }

    int las_broadcastene = 1;
    setsockopt(socket_FD, SOL_SOCKET, SO_BROADCAST, (char *) &las_broadcastene, sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &socket_me, 0, sizeof(socket_me));

    socket_me.sin_family = AF_INET;
    socket_me.sin_port = htons(52999);//toto je port z ktoreho pocuvame
    socket_me.sin_addr.s_addr = htonl(INADDR_ANY);//moze dojst od hocikial..

    socket_send.sin_family = AF_INET;
    socket_send.sin_port = htons(5299);//toto je port na ktory posielame
    socket_send.sin_addr.s_addr = inet_addr(Kconfig::Defaults::ROBOT_IP_ADDRESS.data());//htonl(INADDR_BROADCAST);
    bind(socket_FD, (struct sockaddr *) &socket_me, sizeof(socket_me));

    syslog(LOG_INFO, "[LidarInterface]: Lidar sockets created, send empty command to lidar");

    //najskor posleme prazdny prikaz
    char command = 0x00;
    if (sendto(socket_FD, &command, sizeof(command), 0, (struct sockaddr *) &socket_send, socket_FD_length) == -1)//podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
    {
        syslog(LOG_ERR, "[LidarInterface]: Send empty command failed");
    }

    while (laserDataThreadRun) {
        auto start = chrono::system_clock::now();
        if ((received_length = recvfrom(socket_FD, (char *) &laserData.Data, sizeof(LaserData) * 1000, 0, (struct sockaddr *) &socket_other, &socket_FD_length)) == -1) {
            continue;
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        long int duration = elapsed.count();

        if (duration > 200) {
            syslog(LOG_WARNING, "[LidarInterface]: Laser data not arrived %ld ms", duration);
        }

        start = chrono::system_clock::now();

        laserData.numberOfScans = received_length / sizeof(LaserData);
        updateLocalMap(laserData);

        end = std::chrono::system_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        duration = elapsed.count();

        if (duration > 50) {
            syslog(LOG_WARNING, "[LidarInterface]: Update local map takes so long time: %ld ms", duration);
        }
    }
}

LaserMeasurement LidarInterface::getLaserData() {
    lock_guard<mutex> lockGuard(laserData_mtx);
    return laserData;
}

RobotMap LidarInterface::getRobotMap(){
    localMap_mtx.lock();
    auto map = localMap.getRobotMap();
    localMap_mtx.unlock();

    auto odom = robot->getOdomData();
    map.rotateMap(odom.fi, odom);
    return map;
}

void LidarInterface::updateLocalMap(LaserMeasurement laserData){
    static RobotPose odom_old;
    static auto start = chrono::system_clock::now();

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    auto duration = elapsed.count();
//    syslog(LOG_INFO, "durtion %d", duration);

    RobotPose odom = robot->getOdomData();
    auto distance = robot->getAbsoluteDistance(odom_old, odom);

    if (distance < Kconfig::LidarControl::UPDATE_DISTANCE && duration < Kconfig::LidarControl::UPDATE_DURATION) {
        return;
    }

    odom_old = odom;
    start = end;

    lock_guard<mutex> lk(localMap_mtx);
    localMap.addMeasurementForgetting(odom, &laserData, Kconfig::LidarControl::DATA_HOLD_COEFFICIENT);
}

void LidarInterface::clearMap() {
    lock_guard<mutex> lk(localMap_mtx);
    localMap.clearMap();
}
