#ifndef RPLIDAR_H
#define RPLIDAR_H
#include "rplidar.h"

#include "errno.h"


int
set_interface_attribs (int fd, int speed, int parity)
{
 /*       struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }*/
        return 0;
}

void
set_blocking (int fd, int should_block)
{
   /*     struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);*/
}



int rplidar::connect(char *comport)
{
/*    hCom= open(comport,O_RDWR | O_NOCTTY | O_SYNC);
    if ( hCom== -1 )
    {
        // Chyba:  Port sa neda otvorit


        return -1;
    }
    else
    {

        set_interface_attribs (hCom, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (hCom, 0);                // set no blocking
     /*   struct termios2 tio;
        ioctl(hCom,TCGETS2,&tio);
        tio.c_cflag &=~CBAUD;
        tio.c_cflag |=BOTHER;
        tio.c_ispeed=115200;
        tio.c_ospeed=115200;
        ioctl(hCom,TCSETS2,&tio);*
        printf("mam port %i\n",hCom);


        //broadcast cast
        slen = sizeof(si_other);
        if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
        {

        }

        int broadcastene=1;
        setsockopt(s,SOL_SOCKET,SO_BROADCAST,&broadcastene,sizeof(broadcastene));
        // zero out the structure
        memset((char *) &si_me, 0, sizeof(si_me));

        si_me.sin_family = AF_INET;
        si_me.sin_port = htons(5299);
        si_me.sin_addr.s_addr = htonl(INADDR_ANY);

        si_posli.sin_family = AF_INET;
        si_posli.sin_port = htons(52999);
        si_posli.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    bind(s , (struct sockaddr*)&si_me, sizeof(si_me) );
        //----------------------------------

        return 0;
    }*/
    return -1;
}

//------------

void rplidar::recvCommandUDP()
{
    while(1)
    {

        if ((recv_len = recvfrom(s, buf, 1, 0, (struct sockaddr *) &si_other, &slen)) == -1)
        {

        }
        printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        printf("Data: %s\n" , buf);
        if(buf[0]==1)
        {

        }
        else if(buf[0]==2)
        {

        }
        else if(buf[0]==3)
        {

        }
        else if(buf[0]==4)
        {

        }
        else if(buf[0]==0)
        {

        }
        else if(buf[0]==5)
        {
            //reset spojenia
         /*   stop();
            close(hCom);
            usleep(100*1000);
            WasEnabled=0;
            ktoreMeranie=-1;
            kdeJeCele=-1;
            poslednePoslane=-1;
            ktoreZapisujem=-1;
            ktorePosielam=-1;
            stopMeasurement=0;
            int err=connect("/dev/laser");
            if(err==-1)
            {
                printf("pruuuuser\n");
            }
            enable();
            start();*/
        }

    }
}

//--tu treba dokoncit overenie ci je s laserom vsetko v poriadku.
int rplidar::enable()
{
    WasEnabled=1;
    return 0;
}

int rplidar::start()
{
    if( hCom== -1 )
    {
        printf("lidar nepripojeny\n");
        return -1;
    }
    //--ale ignorujeme teraz
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
    threadID=pthread_create(&threadHandle,NULL,&laserVlakno,(void *)this);
    return threadID;
}

int rplidar::stop()
{
    stopMeasurement=1;
    pthread_join(threadHandle,NULL);
    unsigned char request[]={0xa5, 0x25};
   // int Pocet=0;
    write(hCom,&request,2);

    usleep(2000);
    //tu treba vycitat buffer
    char buffer[5000];
    int readnum=0;
    do
    {
         readnum=read(hCom,buffer,5000);
    }while(readnum>0);

    return 0;
}
LaserMeasurement rplidar::getMeasurement()
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

int rplidar::measure()
{
Start:
    if(stopMeasurement==1)
        return -1;
    //premazem buffer
    unsigned char requestS[]={0xa5, 0x25};
    int Pocet;
    usleep(100*1000);
    Pocet=write(hCom,&requestS,2);
    printf("poslali sme request %i\n",Pocet);
    usleep(100*1000);

    //tu premazat buffer..
    char vystup[2000];
    Pocet=read(hCom,&vystup,2000);
    printf("premazane %i\n",Pocet);
    if(Pocet==-1 || Pocet==2000)
    {
        Pocet=read(hCom,&vystup,2000);
        printf("premazane2 %i\n",Pocet);
    }
    //--poslem request na to aby som zacal merat

    usleep(50*1000);
    unsigned char request[]={0xa5, 0x20};

    Pocet=write(hCom,&request,2);
    usleep(100*1000);
    //--podarilo sa poslat cely prikaz???
    if(Pocet!=2)
    {
        Pocet=write(hCom,&request,2);
        usleep(100*1000);

        //--ak ani po opakovani nefungujeme treba skoncit
        if(Pocet!=2)
        {
            goto Start;
            return -1;
        }
    }
    usleep(100*1000);
    //--ak sme poslali prikaz, treba precitat hlavicku odpovede ma 7 bytov
    unsigned char dataR[7];
    Pocet=read(hCom,dataR,7);

    int error=errno;
    if((dataR[0]!=0xa5)&&(dataR[1]!=0x5a))
    {
        printf("dostali sme tolkoto a je to zle %i,chyba %i\n",Pocet,error);
        printf("nieco je zle, prve vratene byty nesuhlasia s odpovedou vo funkcii measure() %x %x %x %x %x %x %x\n",dataR[0],dataR[1],dataR[2],dataR[3],dataR[4],dataR[5],dataR[6]);
        goto Start;//stopMeasurement=1;
        return -1;
    }
    usleep(50*1000);
    int indexKamPisem=0;
    while(stopMeasurement==0)
    {
        unsigned char dataR[5];
        Pocet=read(hCom,dataR,5);

        int oldP=Pocet;
        if(Pocet!=5)
        {
            usleep(4*1000);
            Pocet=read(hCom,&dataR[Pocet],5-Pocet);
            usleep(1000);
            oldP=oldP+Pocet;
        }
        if(oldP==5)
        {
            //--ci je nove meranie
            if((dataR[0]&0x01)&&( (dataR[0]&0x02) ==0))
            {
                //--podozrenie ze sa nieco dosralo a niesom uplne prvy kus merania
                if((indexKamPisem<15)&&(ktoreZapisujem!=-1)&&(indexKamPisem!=0))
                {
                    printf("nieco sa dosralo\n");
                    goto Start;
                }
                ktoreMeranie++;
                if(ktoreMeranie>-1)
                {
                    kdeJeCele=ktoreZapisujem;
                }
                indexKamPisem=0;
                //---ak bolo -1 aka zaciatok, nastavim ze pisem na 0 miesto, ak uz nejake bolo, zmenime podla toho,kde bolo predtym,a ci nahodou prave neposielam
                ktoreZapisujem= (ktoreZapisujem==-1)? 0: ((ktoreZapisujem+1)%3 ==ktorePosielam)? (ktoreZapisujem+2)%3: (ktoreZapisujem+1)%3;

                //tu preposleme cele meranie na siet...
                if (sendto(s, (char*)&localMeranie[kdeJeCele].Data, sizeof(LaserData)*localMeranie[kdeJeCele].numberOfScans, 0, (struct sockaddr*) &si_posli, slen) == -1)
                {

                }
            }
            unsigned char pomoc=dataR[0];

            localMeranie[ktoreZapisujem].Data[indexKamPisem].scanQuality=pomoc>>2;
            pomoc=dataR[1];
            localMeranie[ktoreZapisujem].Data[indexKamPisem].scanAngle=(float)((float)(pomoc>>1)+(float)(dataR[2]*128))/64;
            localMeranie[ktoreZapisujem].Data[indexKamPisem].scanDistance=(float)((float)(dataR[3])+(float)(dataR[4]*256))/64;
            indexKamPisem++;
            localMeranie[ktoreZapisujem].numberOfScans=indexKamPisem;
        }
        else
        {
            unsigned char requestS[]={0xa5, 0x25};
           // int Pocet=0;
            usleep(100*1000);
            Pocet=write(hCom,&requestS,2);
            usleep(100*1000);

            //mazat buffer
           // int i=PurgeComm(hCom,PURGE_TXCLEAR | PURGE_RXCLEAR);
            usleep(500*1000);
            goto Start;
        }
    }

    return 0;
}

#endif
