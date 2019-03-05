//
// Created by jakub on 13.2.2019.
//

#include <include/robot_interface.h>
#include <functional>

using namespace std;

// vracia znamienko, -1, 1
template <typename T> int signum(T val) {
    return (T(0) < val) - (val < T(0));
}

RobotInterface::RobotInterface() {
    Ki = ROBOT_REG_I;
    Kp = ROBOT_REG_P;
    Kd = ROBOT_REG_D;
    odom = {0};

    robot = thread(&RobotInterface::t_readRobotData, this);
//    laser = thread(&RobotInterface::t_readLaserData, this);

    // start pose controller timer
    std::thread(&RobotInterface::t_poseController, this).detach();
}

RobotInterface::~RobotInterface() {
//    std::terminate();
}


void RobotInterface::t_readRobotData() {
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
    rob_si_posli.sin_addr.s_addr = inet_addr(ipAddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s, (struct sockaddr *) &rob_si_me, sizeof(rob_si_me));

    std::vector<unsigned char> mess = setDefaultPID();
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli, rob_slen) == -1) {

    }
    usleep(100 * 1000);
    mess = setSound(440, 1000);
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli, rob_slen) == -1) {

    }
    unsigned char buff[50000];
    while (1) {
        memset(buff, 0, 50000 * sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char *) &buff, sizeof(char) * 50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1) {

            continue;
        }
        //https://i.pinimg.com/236x/1b/91/34/1b9134e6a5d2ea2e5447651686f60520--lol-funny-funny-shit.jpg
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        //      struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);

//        robotDataMutex.lock();
        int returnval = fillData(robotData, (unsigned char *) buff);
        if (returnval == 0) {
            computeOdometry(robotData.EncoderRight, robotData.EncoderLeft, robotData.GyroAngle);
//            processRobotData = thread(&RobotInterface::t_computeOdometry, this, robotData.EncoderRight, robotData.EncoderLeft, robotData.GyroAngle);
//            processRobotData.join(); // TODO treba domysliet lebo ak vyjde z toho scopu tak thread je terminated
        }
//        robotDataMutex.unlock();
    }
}

void RobotInterface::computeOdometry(unsigned short encoderRight, unsigned short encoderLeft, signed short gyroAngle) {
    //TODO ak je polomer otacania robota mensi ak nieco tak ber uhol otocenia z gyra
    static unsigned short encoderLeftOld, encoderRightOld;
    static signed short gyroAngleOld;

    int encoderLeftDelta;
    int encoderRightDelta;

    // detekcia pretecenia encodera,
    if (abs(encoderLeft - encoderLeftOld) > ROBOT_ENCODER_MAX / 2) {
        if (encoderLeft < encoderLeftOld) {
            encoderLeftDelta = ROBOT_ENCODER_MAX - encoderLeftOld + encoderLeft;
        } else {
            encoderLeftDelta = -(ROBOT_ENCODER_MAX - encoderLeft + encoderLeftOld);
        }
    } else {
        // v pripade bez pretecenia
        encoderLeftDelta = encoderLeft - encoderLeftOld;
    }

    if (abs(encoderRight - encoderRightOld) > ROBOT_ENCODER_MAX / 2) {
        if (encoderRight < encoderRightOld) {
            encoderRightDelta = ROBOT_ENCODER_MAX - encoderRightOld + encoderRight;
        } else {
            encoderRightDelta = -(ROBOT_ENCODER_MAX - encoderRight + encoderRightOld);
        }
    } else {
        // v pripade bez pretecenia
        encoderRightDelta = encoderRight - encoderRightOld;
    }

    // detekcia pretecenie gyra
    double gyroAngleDelta = 0;

    if (forOdomUseGyro) {
        if (abs(gyroAngleOld - gyroAngle) > ROBOT_GYRO_MAX) {
            //detekuj smer pretecenia
            if (gyroAngleOld < gyroAngle) {
                gyroAngleDelta = -ROBOT_GYRO_MAX - gyroAngleOld - (ROBOT_GYRO_MAX - gyroAngle);
            } else {
                gyroAngleDelta = ROBOT_GYRO_MAX - gyroAngleOld + (ROBOT_GYRO_MAX + gyroAngle);
            }
        } else {
            gyroAngleDelta = gyroAngle - gyroAngleOld;
        }
        gyroAngleDelta = gyroAngleDelta / 100 * DEG2RAD;     // prichadza to ako int s tym ze je desatina ciarka posunuta o 2 miesta
    }

    double distanceLeft = encoderLeftDelta * ROBOT_TICK_TO_METER;
    double distanceRight = encoderRightDelta * ROBOT_TICK_TO_METER;
    double distanceCenter = (distanceLeft + distanceRight)/2;

    odom_mtx.lock();

    double fiOld = odom.fi;

    // vypocet natocenia
    if (forOdomUseGyro) {
        odom.fi = odom.fi + gyroAngleDelta;
    } else {
        odom.fi = odom.fi + (distanceRight - distanceLeft) / ROBOT_WHEEL_BASE;
    }

    // osetrit pretocenie robota o 2pi
    if (odom.fi > M_PI) {
        odom.fi -= M_PI * 2;
    } else if (odom.fi < -M_PI) {
        odom.fi += M_PI * 2;
    }

    if (distanceRight - distanceLeft == 0) {
        // ked robot ide priamo
        odom.x = odom.x + distanceCenter;
        odom.y = odom.y + distanceCenter;
    } else {
        // ked robot ide po krivke
        odom.x = odom.x + ((ROBOT_WHEEL_BASE * (distanceRight + distanceLeft)) / (2*(distanceRight - distanceLeft))) * (sin(odom.fi) - sin(fiOld));
        odom.y = odom.y - ((ROBOT_WHEEL_BASE * (distanceRight + distanceLeft)) / (2*(distanceRight - distanceLeft))) * (cos(odom.fi) - cos(fiOld));
    }

    odom_mtx.unlock();

    //TODO treba urobit nejaky signal ktory da info UI o zmene premennej
    //TODO ? https://en.cppreference.com/w/cpp/thread/condition_variable

    gyroAngleOld = gyroAngle;
    encoderLeftOld = encoderLeft;
    encoderRightOld = encoderRight;
}

void RobotInterface::t_readLaserData() {

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
    las_si_posli.sin_addr.s_addr = inet_addr(ipAddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s, (struct sockaddr *) &las_si_me, sizeof(las_si_me));
    char command = 0x00;
    //najskor posleme prazdny prikaz
    //preco?
    //https://ih0.redbubble.net/image.74126234.5567/raf,750x1000,075,t,heather_grey_lightweight_raglan_sweatshirt.u3.jpg
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

//600
std::vector<unsigned char> RobotInterface::setTranslationSpeed(int mmpersec) {
    unsigned char message[14] = {0xaa, 0x55, 0x0A, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04, (unsigned char) (mmpersec % 256),
                                 (unsigned char) (mmpersec >> 8), 0x00, 0x00, 0x00};
    message[13] =
            message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^
            message[10] ^ message[11] ^ message[12];


    std::vector<unsigned char> vystup(message, message + sizeof(message) / sizeof(message[0]));
    return vystup;

}

// 2pi
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

//radius <short> -32000 32000 mm
std::vector<unsigned char> RobotInterface::setArcSpeed(int mmpersec, int radius) {
    if (radius == 0) {
        return setTranslationSpeed(mmpersec);
    }
    //viac o prikaze a jeho tvorbe si mozete precitat napriklad tu
    //http://yujinrobot.github.io/kobuki/enAppendixProtocolSpecification.html

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

void RobotInterface::sendTranslationSpeed(int mmPerSec) {
    std::vector<unsigned char> mess = setTranslationSpeed(mmPerSec);
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli, rob_slen) == -1) {  }
    forOdomUseGyro = false;
}

void RobotInterface::sendRotationSpeed(int radPerSec) {
    std::vector<unsigned char> mess = setRotationSpeed(radPerSec);
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli, rob_slen) == -1) {  }
    forOdomUseGyro = true;
}

void RobotInterface::sendArcSpeed(int mmPerSec, int mmRadius) {
    std::vector<unsigned char> mess = setArcSpeed(mmPerSec, mmRadius);
    if (sendto(rob_s, (char *) mess.data(), sizeof(char) * mess.size(), 0, (struct sockaddr *) &rob_si_posli, rob_slen) == -1) {  }
    forOdomUseGyro = mmRadius < ROBOT_THRESHOLD_RADIUS_GYRO_COMPUTATION;
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
            output.BumperCenter = data[checkedValue] & 0x02;
            output.BumperLeft = data[checkedValue] & 0x04;
            output.BumperRight = data[checkedValue] & 0x01;
            checkedValue++;
            output.WheelDropLeft = data[checkedValue] & 0x02;
            output.WheelDropRight = data[checkedValue] & 0x01;
            checkedValue++;
            output.CliffCenter = data[checkedValue] & 0x02;
            output.CliffLeft = data[checkedValue] & 0x04;
            output.CliffRight = data[checkedValue] & 0x01;
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
    lock_guard<mutex> lockGuard(laserData_mtx);
    return laserData;
}

RobotPose RobotInterface::getOdomData() {
    lock_guard<mutex> lockGuard(odom_mtx);
    return odom;
}

double RobotInterface::wheelPID(double w, double y, double saturation) {
    static double previousError = 0, integral = 0;
    static auto start = std::chrono::system_clock::now();

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_time = end - start;
    double dt = elapsed_time.count();

    double error = w - y;
    integral = integral + error * dt;
    double derivative = (error - previousError)/dt;
    double output = Kp*error + Ki*integral + Kd*derivative;

    previousError = error;
    start = end;

    return (output < saturation) ? output : saturation;
}

//void RobotInterface::goToPosition(const RobotPose &position) {
void RobotInterface::goToPosition(RobotPose position, bool leadingEdge, bool trailingEdge) {
    RobotPose odomPosition = getOdomData();

    double translationError = position.x - odomPosition.x;// TODO = getAbsoluteDistance(odomPosition, position);
    int currentSpeed = 0;
    int currentRadius = 0;
    bool braking = false;

    /// Set speed limit, if leading Edge is set, start from minimal speed, otherwise go with maximal speed
    int speedLimit = leadingEdge ? ROBOT_MIN_SPEED_FORWARD : ROBOT_MAX_SPEED_FORWARD;

    /// Calculate speed stepping according to sampling period
    int speedStep = (int) (ROBOT_ACCELERATION * ROBOT_REG_SAMPLING);

    syslog(LOG_NOTICE, "Going to position x, y, fi: {%.lf, %.lf, %.lf}, distance: %.lf mm.", position.x, position.y, position.fi, translationError);

    while (translationError > ROBOT_REG_ACCURACY){
        odomPosition = getOdomData();

        currentSpeed  = (int) wheelPID(translationError, 0, speedLimit);
        currentRadius = (int) fitRotationRadius(position.fi - odomPosition.fi);

        syslog(LOG_DEBUG, "Remaining distance: %.lf, Calculated speed: %d, Calculated radius: %d", translationError, currentRadius, currentRadius);

        /// Set arc speed from calculated speed and radius
        std::vector<unsigned char> msg = setArcSpeed(currentSpeed, currentRadius);
        sendDataToRobot(msg);

        translationError = position.x - odomPosition.x;//= getAbsoluteDistance(odomPosition, position);

        if(translationError < currentSpeed * 3){
            // TODO implementovat nejaku lepsiu podmienku na brzdenie
            cout << "!!!!!!!!!!!!!!!!!!!!!!!! BRAKING !!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl;
            braking = true;
        }

        if (speedLimit < ROBOT_MAX_SPEED_FORWARD && !braking){
            speedLimit += speedStep;
        } else if (speedLimit > ROBOT_MIN_SPEED_FORWARD && braking){
            speedLimit -= speedStep;
        }
        usleep((__useconds_t) (1000 * 1000 * ROBOT_REG_SAMPLING));
    }

    std::vector<unsigned char> msg = setTranslationSpeed(0);

    sendDataToRobot(msg);

    syslog(LOG_NOTICE, "Reached final position with accuracy of %.lf [mm].", translationError);

}

double RobotInterface::getAbsoluteDistance(RobotPose posA, RobotPose posB)
{
    return sqrt(pow(posA.x - posB.x, 2) + pow(posA.y - posB.y, 2));
}

double RobotInterface::fitRotationRadius(double angle)
{
    static const double coef_a = 1.022896253633420e+03;
    static const double coef_b = - 2.943278473631116;
    static const double coef_c = 1.897709639068426e+04;
    static const double coef_d = - 26.057588318697285;

    return coef_a * exp(coef_b * angle) + coef_c * exp(coef_d * angle);
}

void RobotInterface::addCommandToQueue(const RobotPose &cmd) {
    std::lock_guard<std::mutex> robotCmdPoints_lg(robotCmdPoints_mtx);
    robotCmdPoints.push(cmd);
}


void RobotInterface::t_poseController() {
    /*
     * Stavy:
     * - cakanie na bod
     * - pohyb
     * - obchadzanie prekazky
     * - pauza vykonavania bodu
     * - zrusenie vykonavania bodu
     */
    static int state = 0;

    
    // this loop is like a timer with ROBOT_POSE_CONTROLLER_PERIOD period
    while (true) {
        auto startPeriodTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(ROBOT_POSE_CONTROLLER_PERIOD);
        // TIMER
        {

            switch (state) {
                case 0:
                    robotCmdPoints_mtx.lock();


                    robotCmdPoints_mtx.unlock();
                    break;
                case 1:



                default:
                    break;
            }

            //TODO #include <condition_variable> na signalizovanie bllokacie


        }
        std::this_thread::sleep_until(startPeriodTime);
    }

}
