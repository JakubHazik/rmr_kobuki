//
// Created by martin on 14.3.2019.
//

#include <include/lidar_interface.h>

LidarInterface::LidarInterface() {
    laser = thread(&LidarInterface::t_readLaserData, this);
}

LidarInterface::~LidarInterface() {
//    std::terminate();
}

void LidarInterface::t_readLaserData() {

    // Initialize Winsock

    las_slen = sizeof(las_si_other);
    if ((las_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        syslog(LOG_ERR, "Create socket failed");
    }

    int las_broadcastene = 1;
    setsockopt(las_s, SOL_SOCKET, SO_BROADCAST, (char *) &las_broadcastene, sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);//toto je port z ktoreho pocuvame
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);//moze dojst od hocikial..

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);//toto je port na ktory posielame
    las_si_posli.sin_addr.s_addr = inet_addr(ipAddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s, (struct sockaddr *) &las_si_me, sizeof(las_si_me));

    //najskor posleme prazdny prikaz
    char command = 0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr *) &las_si_posli, las_slen) == -1)//podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
    {

    }
//    LaserMeasurement measure;
    while (1) {
        laserData_mtx.lock();
        if ((las_recv_len = recvfrom(las_s, (char *) &laserData.Data, sizeof(LaserData) * 1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1) {
            laserData_mtx.unlock();
            continue;
        }
        laserData.numberOfScans = las_recv_len / sizeof(LaserData);
        laserData_mtx.unlock();
    }
}

LaserMeasurement LidarInterface::getLaserData() {
    lock_guard<mutex> lockGuard(laserData_mtx);
    return laserData;
}