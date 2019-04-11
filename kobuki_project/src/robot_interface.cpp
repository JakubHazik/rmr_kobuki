//
// Created by jakub on 13.2.2019.
//

#include <include/robot_interface.h>

using namespace std;

// vracia znamienko, -1, 0, 1
template <typename T> int signum(T val) {
    return (T(0) < val) - (val < T(0));
}

RobotPose operator+(const RobotPose &a, const RobotPose &b) {
    return {a.x + b.x, a.y + b.y, a.fi + b.fi};
}

RobotInterface::RobotInterface(): poseRegulator(Kconfig::PoseControl::POSE_CONTROLLER_PERIOD) {
    robotDataRecv = thread(&RobotInterface::t_readRobotData, this);

    // start pose controller timer thread
    poseController = thread(&RobotInterface::t_poseController, this);

    // wait until arrive the first message, next we can reset odom
    usleep(1000 * 1000);
    resetOdom(0, 0, 0);
}

RobotInterface::~RobotInterface() {
    robotDataThreadRun = false;
    poseControllerThreadRun = false;
    robotDataRecv.join();
    poseController.join();
}


void RobotInterface::t_readRobotData() {
    if ((rob_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        syslog(LOG_ERR, "[Robot Interface]: Cannot create socket");
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
    rob_si_posli.sin_addr.s_addr = inet_addr(Kconfig::Defaults::ROBOT_IP_ADDRESS.data());
    rob_slen = sizeof(rob_si_me);
    bind(rob_s, (struct sockaddr *) &rob_si_me, sizeof(rob_si_me));

    std::vector<unsigned char> mess = setDefaultPID();
    sendDataToRobot(mess);

    usleep(100 * 1000);
    mess = setSound(440, 1000);
    sendDataToRobot(mess);

    unsigned char buff[50000];
    while (robotDataThreadRun) {
        memset(buff, 0, 50000 * sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char *) &buff, sizeof(char) * 50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1) {
            // no data received, try again
            continue;
        }
        // tu uz mame data..zavolame si funkciu
        int returnval = fillData(robotData, (unsigned char *) buff);    // parse kobuki message, if message is parsed successful, it returns 0
        if (returnval == 0) {
            computeOdometry(robotData.EncoderRight, robotData.EncoderLeft, robotData.GyroAngle);
        }
    }
}

void RobotInterface::computeOdometry(unsigned short encoderRight, unsigned short encoderLeft, signed short gyroAngle) {
    static unsigned short encoderLeftOld, encoderRightOld;
//    static signed short gyroAngleOld;

    int encoderLeftDelta;
    int encoderRightDelta;

//    syslog(LOG_NOTICE, "ENCODER: left: %d; right: %d; gyro: %d", encoderLeft, encoderRight, gyroAngle );

    // detekcia pretecenia encodera,
    if (abs(encoderLeft - encoderLeftOld) > Kconfig::HW::ENCODER_MAX / 2) {
        if (encoderLeft < encoderLeftOld) {
            encoderLeftDelta = Kconfig::HW::ENCODER_MAX - encoderLeftOld + encoderLeft;
        } else {
            encoderLeftDelta = -(Kconfig::HW::ENCODER_MAX - encoderLeft + encoderLeftOld);
        }
    } else {
        // v pripade bez pretecenia
        encoderLeftDelta = encoderLeft - encoderLeftOld;
    }

    if (abs(encoderRight - encoderRightOld) > Kconfig::HW::ENCODER_MAX / 2) {
        if (encoderRight < encoderRightOld) {
            encoderRightDelta = Kconfig::HW::ENCODER_MAX - encoderRightOld + encoderRight;
        } else {
            encoderRightDelta = -(Kconfig::HW::ENCODER_MAX - encoderRight + encoderRightOld);
        }
    } else {
        // v pripade bez pretecenia
        encoderRightDelta = encoderRight - encoderRightOld;
    }

    double distanceLeft = encoderLeftDelta * Kconfig::HW::TICK_TO_METER;
    double distanceRight = encoderRightDelta * Kconfig::HW::TICK_TO_METER;
    double distanceCenter = (distanceLeft + distanceRight)/2;

    odom_mtx.lock();

    double fiOld = odom.fi;

    odom.fi = odom.fi + (distanceRight - distanceLeft) / Kconfig::HW::WHEEL_BASE;

    // osetrit pretocenie robota o 2pi
    if (odom.fi > M_PI) {
        odom.fi -= M_PI * 2;
    } else if (odom.fi < -M_PI) {
        odom.fi += M_PI * 2;
    }

    if (distanceRight - distanceLeft == 0) {
        // ked robot ide priamo
        odom.x = odom.x + distanceCenter * cos(odom.fi);
        odom.y = odom.y + distanceCenter * sin(odom.fi);
    } else {
        // ked robot ide po krivke
        odom.x = odom.x + ((Kconfig::HW::WHEEL_BASE * (distanceRight + distanceLeft)) / (2*(distanceRight - distanceLeft))) * (sin(odom.fi) - sin(fiOld));
        odom.y = odom.y - ((Kconfig::HW::WHEEL_BASE * (distanceRight + distanceLeft)) / (2*(distanceRight - distanceLeft))) * (cos(odom.fi) - cos(fiOld));
    }

    odom_mtx.unlock();

    encoderLeftOld = encoderLeft;
    encoderRightOld = encoderRight;
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
//        double result = mmpersec * 2 * M_PI / 1200;
//        syslog(LOG_NOTICE, "rotation speed:  %f", result);
//        return setRotationSpeed(result);
        radius = 1; // nastavime co najmensi radius 1 mm
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
//    syslog(LOG_INFO, "Setting translation speed to %d [mm/s]", mmPerSec);
    std::vector<unsigned char> mess = setTranslationSpeed(mmPerSec);
    sendDataToRobot(mess);
}

void RobotInterface::sendRotationSpeed(int radPerSec) {
//    syslog(LOG_INFO, "Setting rotation speed to %d [rad/s]", radPerSec);
    std::vector<unsigned char> mess = setRotationSpeed(radPerSec);
    sendDataToRobot(mess);
}

void RobotInterface::sendArcSpeed(int mmPerSec, int mmRadius) {
    /*
     * zaporny radius znamena ze bod otacania je napravo od robota
     */
    if (mmRadius == 0) {
        syslog(LOG_ERR, "[Robot Interface]: Radius have so low value: 0");
    }

    std::vector<unsigned char> mess = setArcSpeed(mmPerSec, mmRadius);
    sendDataToRobot(mess);
}

bool RobotInterface::sendDataToRobot(const std::vector<unsigned char> &mess) {
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {
        syslog(LOG_ERR, "[Robot Interface]: Send data to robot failed!");
        return false;
    }
    return true;
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

RobotPose RobotInterface::getOdomData() {
    lock_guard<mutex> lockGuard(odom_mtx);
    return odom;
}

double RobotInterface::getAbsoluteDistance(RobotPose posA, RobotPose posB) {
    return sqrt(pow(posA.x - posB.x, 2) + pow(posA.y - posB.y, 2));
}

void RobotInterface::t_poseController() {
    static RobotPose poseToGoOld;

    double translationError;
    RobotPose poseToGo;
    bool zoneNotified = false;
    bool goalNotified = false;


    // this loop is like a timer with ROBOT_POSE_CONTROLLER_PERIOD period
    while (poseControllerThreadRun) {
        auto startPeriodTime = std::chrono::steady_clock::now() + std::chrono::milliseconds((int) (Kconfig::PoseControl::POSE_CONTROLLER_PERIOD * 1000));

        // TIMER
        {
            // read target position
            goalPose_mtx.lock();
            poseToGo = this->goalPose;
            goalPose_mtx.unlock();

            if (zoneNotified) {
                if (__glibc_unlikely(!(poseToGo == poseToGoOld))) {
                    zoneNotified = false;
                    goalNotified = false;
                }
            }

            RobotPose odomPosition = getOdomData();
            translationError = getAbsoluteDistance(odomPosition, poseToGo);

            if (__glibc_unlikely(translationError < goalZone && !zoneNotified)) {
                // zona je dosiahnuta
                zoneAchieved.set_value();
                zoneNotified = true;
                syslog(LOG_NOTICE, "[Robot Interface]: Zone achieved");
            }

            // zistovanie ci robot dosiahol bod
            if (__glibc_unlikely(translationError < Kconfig::PoseControl::GOAL_ACCURACY &&  !goalNotified)) {
                // bod je dosiahnuty
                sendTranslationSpeed(0);
                goalAchieved.set_value();
                goalNotified = true;
                syslog(LOG_NOTICE, "[Robot Interface]: Goal achieved");
                continue;
            }

            RegulatorAction regulatorAction = poseRegulator.getAction(odomPosition, poseToGo);
            //cout<<"transError: "<<translationError<<", reg: "<<regulatorAction.radius << ", "<<regulatorAction.speed<<endl;
            sendArcSpeed(regulatorAction.speed, regulatorAction.radius);

            poseToGoOld = poseToGo;
            std::this_thread::sleep_until(startPeriodTime);
        }
    }
}

bool RobotInterface::isGoalAchieved() {
    goalPose_mtx.lock();
    RobotPose poseToGo = this->goalPose;
    goalPose_mtx.unlock();
    double translationError = getAbsoluteDistance(getOdomData(), poseToGo);
    return translationError < Kconfig::PoseControl::GOAL_ACCURACY;
}

void RobotInterface::resetOdom(double x, double y, double fi) {
    std::lock_guard<std::mutex> odom_lg(odom_mtx);
    odom.x = x;
    odom.y = y;
    odom.fi = fi;
}

std::future<void> RobotInterface::setRequiredPose(RobotPose goalPose) {
    lock_guard<mutex> lk(goalPose_mtx);
    this->goalPose = goalPose;
    this->goalAchieved = std::promise<void>();
    return this->goalAchieved.get_future();
}

std::future<void> RobotInterface::setZoneParams(int goalZone) {
    lock_guard<mutex> lk(zone_mtx);
    this->goalZone = goalZone;
    this->zoneAchieved = std::promise<void>();
    return this->zoneAchieved.get_future();
}

std::future<void> RobotInterface::setRequiredPoseOffset(RobotPose goalPose, SPACE space) {
    if (space == ROBOT_SPACE) {
        return setRequiredPose(robot2originSpace(getOdomData(), goalPose));
    } else if (space == ORIGIN_SPACE) {
        return setRequiredPose(goalPose);
    }

    throw std::invalid_argument("scape has no match case");
}

RobotPose RobotInterface::robot2originSpace(RobotPose odom, RobotPose goal) {
    double x = goal.x * cos(odom.fi * DEG2RAD) - goal.y + sin(odom.fi * DEG2RAD) + odom.x;
    double y = goal.x * sin(odom.fi * DEG2RAD) + goal.y + cos(odom.fi * DEG2RAD) + odom.y;
    return {x, y, 0};
}

