#include <include/lidar.h>

Lidar::Lidar() {
    WasEnabled=0;
    ktoreMeranie=-1;
    kdeJeCele=-1;
    poslednePoslane=-1;
    ktoreZapisujem=-1;
    ktorePosielam=-1;
    stopMeasurement=0;

    start();
}

void Lidar::receiveCommandUDP()
{
    while(1)
    {
        if ((received_length = recvfrom(socket_FD, buf, 1, 0, (struct sockaddr *) &socket_other, &socket_FD_length)) == -1)
        {
            syslog(LOG_ERR, "[Lidar]: Receive from fd: %d failed", socket_FD);
        }

        syslog(LOG_NOTICE, "[Lidar]: Received packet from %s:%d\n", inet_ntoa(socket_other.sin_addr), ntohs(socket_other.sin_port));
        syslog(LOG_NOTICE, "[Lidar]: Data: %s\n" , buf);
    }
}


int Lidar::start()
{
    if( hCom== -1 )
    {
        syslog(LOG_ERR, "[Lidar]: Lidar is not connected");
        return -1;
    }

    if( WasEnabled== 0 )
    {
        return -2;
    }
    ktoreMeranie=-2;
    kdeJeCele=-1;
    poslednePoslane=-1;
    ktoreZapisujem=-1;
    ktorePosielam=-1;
    stopMeasurement=0;

    syslog(LOG_INFO, "[Lidar]: Lidar connected, create lidar read thread");

    threadID=pthread_create(&threadHandle,nullptr,&lidarThread,(void *)this);
    return threadID;
}

int Lidar::stop()
{
    stopMeasurement=1;
    pthread_join(threadHandle,nullptr);
    unsigned char request[]={0xa5, 0x25};
    // int Pocet=0;
    write(hCom,&request,2);

    usleep(2000);
    //tu treba vycitat buffer
    char buffer[5000];
    ssize_t readnum=0;
    do
    {
        readnum=read(hCom,buffer,5000);
    }while(readnum>0);

    return 0;
}
LaserMeasurement Lidar::getMeasurement()
{
    LaserMeasurement tempL;
    if(stopMeasurement==1)
    {
        tempL.numberOfScans=-4;
        return tempL;
    }
    if( hCom== -1 )
    {
        tempL.numberOfScans=-3;
        return tempL;
    }
    if(WasEnabled==0)
    {
        tempL.numberOfScans=-2;
        return tempL;
    }
    if(poslednePoslane>=ktoreMeranie)
    {
        tempL.numberOfScans=-1;
        return tempL;
    }

    ktorePosielam=kdeJeCele;
    memcpy(&tempL,&localMeranie[kdeJeCele],sizeof(LaserMeasurement));
    poslednePoslane=ktoreMeranie;
    ktorePosielam=-1;

    return tempL;
}

int Lidar::measure()
{
    Start:
    if(stopMeasurement == 1){
        syslog(LOG_INFO, "[Lidar]: Measure stopped by user.");
        return -1;
    }

    /// Clear buffer
    unsigned char requestS[] = {0xa5, 0x25};
    ssize_t Pocet;
    usleep(100 * 1000);
    Pocet = write(hCom, &requestS, 2);
    syslog(LOG_DEBUG, "[Lidar]: Request sent: %zd\n", Pocet);
    usleep(100 * 1000);

    /// Clear buffer again
    char vystup[2000];
    Pocet = read(hCom, &vystup, 2000);
    syslog(LOG_DEBUG, "[Lidar]: Cleared:  %zd\n", Pocet);

    if(Pocet == -1 || Pocet == 2000)
    {
        Pocet = read(hCom, &vystup, 2000);
        syslog(LOG_DEBUG, "[Lidar]: Cleared (2): %zd\n", Pocet);
    }

    /// Send request to start measure
    usleep(50 * 1000);
    unsigned char request[] = {0xa5, 0x20};

    Pocet = write(hCom, &request, 2);
    usleep(100 * 1000);

    /// Check if whole command was sent
    if(Pocet != 2)
    {
        /// Resend command
        Pocet = write(hCom, &request, 2);
        usleep(100 * 1000);

        /// Stop
        if(Pocet != 2)
        {
            goto Start;
        }
    }
    usleep(100*1000);

    /// Command was sent, now read 7 bytes header
    unsigned char dataR[7];
    Pocet = read(hCom, dataR, 7);

    if((dataR[0] != 0xa5)&&(dataR[1] != 0x5a))
    {
        syslog(LOG_ERR, "[Lidar]: Received %zd bytes (expected 7) - %i", Pocet, errno);
        syslog(LOG_ERR, "[Lidar]: nieco je zle, prve vratene byty nesuhlasia s odpovedou vo funkcii measure() %x %x %x %x %x %x %x\n",dataR[0],dataR[1],dataR[2],dataR[3],dataR[4],dataR[5],dataR[6]);
        goto Start;
    }
    usleep(50*1000);

    int indexKamPisem=0;
    while(stopMeasurement == 0)
    {
        unsigned char dataRead[5];
        Pocet = read(hCom, dataRead, 5);

        ssize_t oldP = Pocet;
        if(Pocet != 5)
        {
            usleep(4*1000);
            Pocet = read(hCom, &dataR[Pocet], 5 - (size_t)Pocet);
            usleep(1000);
            oldP = oldP + Pocet;
        }
        if(oldP == 5)
        {
            /// Check new measurement
            if((dataR[0]&0x01)&&( (dataR[0]&0x02) == 0))
            {
                //--podozrenie ze sa nieco dosralo a niesom uplne prvy kus merania
                if((indexKamPisem < 15) && (ktoreZapisujem != -1) && (indexKamPisem != 0))
                {
                    syslog(LOG_ERR, "[Lidar]: Something went wrong");
                    goto Start;
                }

                ktoreMeranie++;
                if(ktoreMeranie >- 1)
                {
                    kdeJeCele = ktoreZapisujem;
                }

                indexKamPisem = 0;
                //---ak bolo -1 aka zaciatok, nastavim ze pisem na 0 miesto, ak uz nejake bolo, zmenime podla toho,kde bolo predtym,a ci nahodou prave neposielam
                ktoreZapisujem = (ktoreZapisujem == -1) ? 0 : ((ktoreZapisujem + 1) % 3 == ktorePosielam) ? (ktoreZapisujem + 2) % 3: (ktoreZapisujem + 1) % 3;

                //tu preposleme cele meranie na siet...
                if (sendto(socket_FD, (char*)&localMeranie[kdeJeCele].Data, sizeof(LaserData)*localMeranie[kdeJeCele].numberOfScans, 0, (struct sockaddr*) &socket_send, socket_FD_length) == -1)
                {
                    syslog(LOG_ERR, "[Lidar]: Send failed (%d)", socket_FD);
                }
            }
            unsigned char pomoc = dataR[0];

            localMeranie[ktoreZapisujem].Data[indexKamPisem].scanQuality=pomoc>>2;
            pomoc=dataR[1];
            localMeranie[ktoreZapisujem].Data[indexKamPisem].scanAngle=(float)((float)(pomoc>>1)+(float)(dataR[2]*128))/64;
            localMeranie[ktoreZapisujem].Data[indexKamPisem].scanDistance=(float)((float)(dataR[3])+(float)(dataR[4]*256))/64;
            indexKamPisem++;
            localMeranie[ktoreZapisujem].numberOfScans=indexKamPisem;
        }
        else
        {
            unsigned char requestSend[] = {0xa5, 0x25};
            usleep(100 * 1000);
            Pocet = write(hCom, &requestSend ,2);
            usleep(100 * 1000);
            usleep(500 * 1000);
            goto Start;
        }
    }
    return 0;
}