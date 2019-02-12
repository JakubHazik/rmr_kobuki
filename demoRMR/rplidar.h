////*************************************************************************************
////*************************************************************************************
//// autor Martin Dekan  mail: dekdekan@gmail.com
////-------------------------------------------------------------------------------------
//// co to je:
//// trieda ktora je schopna pracovat s laserovym dialkomerom RPlidar (firma robopeak)
//// implementovana funkcnost je znacne obmedzena a obsahuje len fukncie ktore som bol
//// schopny vytvorit za jeden den, ked som nemal nic lepsie na praci
//// treba ratat s tym,ze niesu osetrene chybove stavy zariadenia
////*************************************************************************************
////*************************************************************************************
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include "unistd.h"
#include "pthread.h"
#include "iostream"
#include "fcntl.h"
#include "string.h"
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#include<iostream>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>

typedef struct
{
    int scanQuality;
    double scanAngle;
    double scanDistance;
}LaserData;

typedef struct
{
    int numberOfScans;
    LaserData Data[1000];
}LaserMeasurement;
class rplidar
{
public:
    int i;
    rplidar()
    {
        WasEnabled=0;
        ktoreMeranie=-1;
        kdeJeCele=-1;
        poslednePoslane=-1;
        ktoreZapisujem=-1;
        ktorePosielam=-1;
        stopMeasurement=0;
    }

     rplidar(char *comport)
     {
         WasEnabled=0;
         ktoreMeranie=-1;
         kdeJeCele=-1;
         poslednePoslane=-1;
         ktoreZapisujem=-1;
         ktorePosielam=-1;
         stopMeasurement=0;
         int err=connect(comport);
         if(err==-1)
         {
             printf("pruuuuser\n");
         }
         enable();
         start();
     }
    virtual  ~rplidar()
    {

    }
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
    struct sockaddr_in si_me, si_other,si_posli;

        int s,  recv_len;
        unsigned int slen;
        char buf[50000];
        void recvCommandUDP();
        //------------------
    // pripoji nas na laserovy dialkomer
    int connect(char *comport);
    //skontroluje funkcnost dialkomeru.. vhodne zavolat pred startom..
    //
    int enable();
    //--spustim meranie laserovym dialkomerom
    int start();
    //--stopnem meranie laserovym dialkomerom
    int stop();
    //--funkcia ktora v nekonecnom vlakne meria a naplna struktury
    int measure();
    //--v pripade ak uzivatel chce nejaku neimplementovanu funkcnost (zrusi meranie ak bezi, vycisti buffer a posle request, vrati co poslal laser)
    unsigned char getUnspecifiedResponse(unsigned char *request);
    //vrati nam aktualne meranie ak vzniklo nove, ak nieje nove meranie v numberOfScans je -1; ak nebolo inicializovane tak -2. ak nieje funkcne spojenie tak -3,ak nieje spustene meranie -4
    LaserMeasurement getMeasurement();
 int vystupvlakno;
private:

    //--spustenie merania v novom vlakne (vycitavanie bezi v novom vlakne. treba ho stopnut ak chceme poslat request)
    static void *laserVlakno(void *param)
    {
        rplidar *rplid=(rplidar*)param;
        rplid->vystupvlakno=rplid->measure();

        return &(rplid->vystupvlakno);
    }

};
