//
// Created by martin on 19.3.2019.
//

#ifndef KOBUKI_PROJECT_LIDAR_H
#define KOBUKI_PROJECT_LIDAR_H

#pragma once

#include <mutex>
#include <thread>
#include <syslog.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <include/own_typedefs.h>

class Lidar {
public:
    Lidar();

    Lidar(char *comport)
    {
        WasEnabled=0;
        ktoreMeranie=-1;
        kdeJeCele=-1;
        poslednePoslane=-1;
        ktoreZapisujem=-1;
        ktorePosielam=-1;
        stopMeasurement=0;

        start();
    }
    virtual ~Lidar() = default;
private:
    //-ci bolo skontrolovane ze je vsetko ok
    int WasEnabled;
    int stopMeasurement;
    //--interne uchovane meranie poslednych dvoch a posielame aktualne (poslednych dvoch,aby sme mohli zapisovat kym posielame)
    LaserMeasurement localMeranie[3];
    //--ze ktore aktualne mame zapisane
    long long ktoreMeranie;
    //--ktore pole ma pouzit na poslanie
    int kdeJeCele;
    int ktorePosielam;
    int ktoreZapisujem;
    //--ktore je posledne odovzdane uzivatelovi
    long long poslednePoslane;
    int hCom;
    pthread_t threadHandle; // handle na vlakno
    int threadID;  // id vlakna

public:

    //veci na broadcast
    struct sockaddr_in socket_me, socket_other, socket_send;

    int socket_FD;
    ssize_t received_length;

    unsigned int socket_FD_length;
    char buf[50000];
    void receiveCommandUDP();


    //--spustim meranie laserovym dialkomerom
    int start();
    //--stopnem meranie laserovym dialkomerom
    int stop();
    //--funkcia ktora v nekonecnom vlakne meria a naplna struktury
    int measure();

    //vrati nam aktualne meranie ak vzniklo nove, ak nieje nove meranie v numberOfScans je -1; ak nebolo inicializovane tak -2. ak nieje funkcne spojenie tak -3,ak nieje spustene meranie -4
    LaserMeasurement getMeasurement();
    int vystupvlakno;
private:

    //--spustenie merania v novom vlakne (vycitavanie bezi v novom vlakne. treba ho stopnut ak chceme poslat request)
    static void *lidarThread(void *param)
    {
        auto *rplid=(Lidar*)param;
        rplid->vystupvlakno=rplid->measure();

        return &(rplid->vystupvlakno);
    }

};

#endif //KOBUKI_PROJECT_LIDAR_H
