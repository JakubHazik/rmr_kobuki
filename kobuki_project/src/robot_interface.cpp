//
// Created by jakub on 13.2.2019.
//

#include <include/robot_interface.h>

#include "../include/robot_interface.h"

using namespace std;

RobotInterface::RobotInterface() {
    thread robot(&RobotInterface::t_processRobotData, this);
    thread laser(&RobotInterface::t_processLaserData, this);

}

void RobotInterface::t_processRobotData() {
    if ((rob_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {

    }

    char rob_broadcastene = 1;
    setsockopt(rob_s, SOL_SOCKET, SO_BROADCAST, &rob_broadcastene, sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s, (struct sockaddr *) &rob_si_me, sizeof(rob_si_me));

    std::vector<unsigned char> mess = setDefaultPID();
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli,
               rob_slen) == -1) {

    }
    usleep(100 * 1000);
    mess = setSound(440, 1000);
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli,
               rob_slen) == -1) {

    }
    unsigned char buff[50000];
    while (1) {
        memset(buff, 0, 50000 * sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char *) &buff, sizeof(char) * 50000, 0, (struct sockaddr *) &rob_si_other,
                                     &rob_slen)) == -1) {

            continue;
        }
        //https://i.pinimg.com/236x/1b/91/34/1b9134e6a5d2ea2e5447651686f60520--lol-funny-funny-shit.jpg
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        //      struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

        int returnval = fillData(robotdata, (unsigned char *) buff);
        if (returnval == 0) {
            //TODO signal ze data su prijate, alebo lepsie vytvorit tu thread na spracovanie dat (odometria)
//            processThisRobot();
        }


    }
}

void RobotInterface::t_processLaserData() {

    // Initialize Winsock

    las_slen = sizeof(las_si_other);
    if ((las_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {

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
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s, (struct sockaddr *) &las_si_me, sizeof(las_si_me));
    char command = 0x00;
    //najskor posleme prazdny prikaz
    //preco?
    //https://ih0.redbubble.net/image.74126234.5567/raf,750x1000,075,t,heather_grey_lightweight_raglan_sweatshirt.u3.jpg
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr *) &las_si_posli, las_slen) ==
        -1)//podla toho vie kam ma robot posielat udaje-odtial odkial mu dosla posledna sprava
    {

    }
//    LaserMeasurement measure;
    while (1) {

        laserDataMutex.lock();
        if ((las_recv_len = recvfrom(las_s, (char *) &laserData.Data, sizeof(LaserData) * 1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1) {
            laserDataMutex.unlock();
            continue;
        }
        laserData.numberOfScans = las_recv_len / sizeof(LaserData);
        laserDataMutex.unlock();

    }
}


std::vector<unsigned char> RobotInterface::setTranslationSpeed(int mmpersec) {
    unsigned char message[14] = {0xaa, 0x55, 0x0A, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04, (unsigned char) (mmpersec % 256),
                                 (unsigned char) (mmpersec >> 8), 0x00, 0x00, 0x00};
    message[13] =
            message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^
            message[10] ^ message[11] ^ message[12];


    std::vector<unsigned char> vystup(message, message + sizeof(message) / sizeof(message[0]));
    return vystup;

}

std::vector<unsigned char> RobotInterface::setRotationSpeed(double radpersec) {
    int speedvalue = radpersec * 230.0f / 2.0f;
    unsigned char message[14] = {0xaa, 0x55, 0x0A, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04,
                                 (unsigned char) (speedvalue % 256), (unsigned char) (speedvalue >> 8), 0x01, 0x00,
                                 0x00};
    message[13] =
            message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^
            message[10] ^ message[11] ^ message[12];


    std::vector<unsigned char> vystup(message, message + sizeof(message) / sizeof(message[0]));
    return vystup;
}

std::vector<unsigned char> RobotInterface::setArcSpeed(int mmpersec, int radius) {
    if (radius == 0) {
        return setTranslationSpeed(mmpersec);

    }
    //viac o prikaze a jeho tvorbe si mozete precitat napriklad tu
    //http://yujinrobot.github.io/kobuki/enAppendixProtocolSpecification.html
    //alebo tu
    //https://bit.ly/2MWSbcx

    int speedvalue = mmpersec * ((radius + (radius > 0 ? 230 : -230)) / 2) / radius;

    unsigned char message[14] = {0xaa, 0x55, 0x0A, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04,
                                 (unsigned char) (speedvalue % 256), (unsigned char) (speedvalue >> 8),
                                 (unsigned char) (radius % 256), (unsigned char) (radius >> 8), 0x00};
    message[13] =
            message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^
            message[10] ^ message[11] ^ message[12];

    std::vector<unsigned char> vystup(message, message + sizeof(message) / sizeof(message[0]));
    return vystup;
}

///2 body navyse prvej skupine ktora pomocou tejto funkcie zahra melodiu pink panther (staci 5 sekund)
/// druha skupina co chce 2 body musi zahrat uvod zo smooth criminal
/// neuverite, 2body moze ziskat aj tretia skupina.. jedine co preto musi spravit je zahrat na robote Bohemian Rhapsody (kompletnu pesnicku.aj s vokalmi)
std::vector<unsigned char> RobotInterface::setSound(int noteinHz, int duration) {
    int notevalue = floor((double) 1.0 / ((double) noteinHz * 0.00000275) + 0.5);
    unsigned char message[13] = {0xaa, 0x55, 0x09, 0x0c, 0x02, 0xf0, 0x00, 0x03, 0x03,
                                 (unsigned char) (notevalue % 256), (unsigned char) (notevalue >> 8),
                                 (unsigned char) (duration % 256), 0x00};
    message[12] =
            message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^
            message[10] ^ message[11];

    std::vector<unsigned char> vystup(message, message + sizeof(message) / sizeof(message[0]));
    return vystup;
}

std::vector<unsigned char> RobotInterface::setDefaultPID() {
    unsigned char message[23] = {0xaa, 0x55, 0x13, 0x0c, 0x02, 0xf0, 0x00, 0x0D, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    message[22] = 0;
    for (int i = 0; i < 23 - 3; i++) {
        message[22] = message[22] ^ message[i + 2];
    }


    std::vector<unsigned char> vystup(message, message + sizeof(message) / sizeof(message[0]));
    return vystup;
}

void RobotInterface::sendTranslationSpeed(double mmPerSec) {
    std::vector<unsigned char> mess = setTranslationSpeed((int) round(mmPerSec));
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli,
               rob_slen) == -1) {}
}

void RobotInterface::sendRotationSpeed(double radPerSec) {
    std::vector<unsigned char> mess = setRotationSpeed((int) round(radPerSec));
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli,
               rob_slen) == -1) {

    }
}

void RobotInterface::sendArcSpeed(double mmPerSec, double radius) {
    std::vector<unsigned char> mess = setArcSpeed((int) round(mmPerSec), (int) round(radius));
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli,
               rob_slen) == -1) {

    }
}

int RobotInterface::checkChecksum(unsigned char *data) {//najprv hlavicku
    unsigned char chckSum = 0;
    for (int i = 0; i < data[0] + 2; i++) {
        chckSum ^= data[i];
    }
    return chckSum;//0 ak je vsetko v poriadku,inak nejake cislo
}

int RobotInterface::parseKobukiMessage(TKobukiData &output, unsigned char *data) {

    int rtrnvalue = checkChecksum(data);
    //ak je zly checksum,tak kaslat na to
    if (rtrnvalue != 0)
        return -2;

    int checkedValue = 1;
    //kym neprejdeme celu dlzku
    while (checkedValue < data[0]) {
        //basic data subload
        if (data[checkedValue] == 0x01) {
            checkedValue++;
            if (data[checkedValue] != 0x0F)
                return -1;
            checkedValue++;
            output.timestamp = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.BumperCenter = data[checkedValue] && 0x02;
            output.BumperLeft = data[checkedValue] && 0x04;
            output.BumperRight = data[checkedValue] && 0x01;
            checkedValue++;
            output.WheelDropLeft = data[checkedValue] && 0x02;
            output.WheelDropRight = data[checkedValue] && 0x01;
            checkedValue++;
            output.CliffCenter = data[checkedValue] && 0x02;
            output.CliffLeft = data[checkedValue] && 0x04;
            output.CliffRight = data[checkedValue] && 0x01;
            checkedValue++;
            output.EncoderLeft = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.EncoderRight = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.PWMleft = data[checkedValue];
            checkedValue++;
            output.PWMright = data[checkedValue];
            checkedValue++;
            output.ButtonPress = data[checkedValue];
            checkedValue++;
            output.Charger = data[checkedValue];
            checkedValue++;
            output.Battery = data[checkedValue];
            checkedValue++;
            output.overCurrent = data[checkedValue];
            checkedValue++;
        } else if (data[checkedValue] == 0x03) {
            checkedValue++;
            if (data[checkedValue] != 0x03)
                return -3;
            checkedValue++;
            output.IRSensorRight = data[checkedValue];
            checkedValue++;
            output.IRSensorCenter = data[checkedValue];
            checkedValue++;
            output.IRSensorLeft = data[checkedValue];
            checkedValue++;
        } else if (data[checkedValue] == 0x04) {
            checkedValue++;
            if (data[checkedValue] != 0x07)
                return -4;
            checkedValue++;
            output.GyroAngle = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.GyroAngleRate = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 5;//3 unsued
        } else if (data[checkedValue] == 0x05) {
            checkedValue++;
            if (data[checkedValue] != 0x06)
                return -5;
            checkedValue++;
            output.CliffSensorRight = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.CliffSensorCenter = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.CliffSensorLeft = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
        } else if (data[checkedValue] == 0x06) {
            checkedValue++;
            if (data[checkedValue] != 0x02)
                return -6;
            checkedValue++;
            output.wheelCurrentLeft = data[checkedValue];
            checkedValue++;
            output.wheelCurrentRight = data[checkedValue];
            checkedValue++;

        } else if (data[checkedValue] == 0x0A) {
            checkedValue++;
            if (data[checkedValue] != 0x04)
                return -7;
            checkedValue++;
            output.extraInfo.HardwareVersionPatch = data[checkedValue];
            checkedValue++;
            output.extraInfo.HardwareVersionMinor = data[checkedValue];
            checkedValue++;
            output.extraInfo.HardwareVersionMajor = data[checkedValue];
            checkedValue += 2;

        } else if (data[checkedValue] == 0x0B) {
            checkedValue++;
            if (data[checkedValue] != 0x04)
                return -8;
            checkedValue++;
            output.extraInfo.FirmwareVersionPatch = data[checkedValue];
            checkedValue++;
            output.extraInfo.FirmwareVersionMinor = data[checkedValue];
            checkedValue++;
            output.extraInfo.FirmwareVersionMajor = data[checkedValue];
            checkedValue += 2;

        } else if (data[checkedValue] == 0x0D) {
            checkedValue++;
            if (data[checkedValue] % 2 != 0)
                return -9;
            checkedValue++;
            output.frameId = data[checkedValue];
            checkedValue++;
            int howmanyFrames = data[checkedValue] / 3;
            checkedValue++;
            output.gyroData.reserve(howmanyFrames);
            output.gyroData.clear();
            for (int hk = 0; hk < howmanyFrames; hk++) {
                TRawGyroData temp;
                temp.x = data[checkedValue + 1] * 256 + data[checkedValue];
                checkedValue += 2;
                temp.y = data[checkedValue + 1] * 256 + data[checkedValue];
                checkedValue += 2;
                temp.z = data[checkedValue + 1] * 256 + data[checkedValue];
                checkedValue += 2;
                output.gyroData.push_back(temp);
            }
        } else if (data[checkedValue] == 0x10) {
            checkedValue++;
            if (data[checkedValue] != 0x10)
                return -10;
            checkedValue++;
            output.digitalInput = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh0 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh1 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh2 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh3 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 8;//2+6


        } else if (data[checkedValue] == 0x13) {
            checkedValue++;
            if (data[checkedValue] != 0x0C)
                return -11;
            checkedValue++;
            output.extraInfo.UDID0 = data[checkedValue + 3] * 256 * 256 * 256 + data[checkedValue + 2] * 256 * 256 +
                                     data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 4;
            output.extraInfo.UDID1 = data[checkedValue + 3] * 256 * 256 * 256 + data[checkedValue + 2] * 256 * 256 +
                                     data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 4;
            output.extraInfo.UDID2 = data[checkedValue + 3] * 256 * 256 * 256 + data[checkedValue + 2] * 256 * 256 +
                                     data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 4;
        } else {
            checkedValue++;
            checkedValue += data[checkedValue] + 1;
        }
    }
    return 0;
}

LaserMeasurement RobotInterface::getLaserData() {
    LaserMeasurement data;

    laserDataMutex.lock();
    memcpy(&data, &laserData, sizeof(LaserMeasurement));
    laserDataMutex.unlock();

    return data;
}


