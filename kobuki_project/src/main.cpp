//
// Created by jakub on 13.2.2019.
//

#include "include/ui.h"
#include "include/robot_interface.h"
#include <QApplication>


using namespace std;

int main(int argc, char *argv[]) {
    openlog("kobuki", LOG_PERROR, LOG_DAEMON);
    syslog(LOG_INFO, "Kobuki started!");
//    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();
//
//    return a.exec();

    RobotInterface robot;



    usleep(1000*1000*3);
    robot.resetOdom();

    auto data = robot.getOdomData();


//    robot.addCommandToQueue({-1000, 0, 0});

//    robot.addCommandToQueue({-1000, 0, 0});
    robot.addCommandToQueue({0 , 0, 0 * DEG2RAD});
    robot.addCommandToQueue({1000, 0, 0 * DEG2RAD});
    robot.addCommandToQueue({1000, 1000, 0 * DEG2RAD});
    robot.addCommandToQueue({0, 1000, 0 * DEG2RAD});
    robot.addCommandToQueue({0, 0, 0 * DEG2RAD});
    robot.addCommandToQueue({1000, 1000, 0 * DEG2RAD});
    robot.addCommandToQueue({0, 0, 0 * DEG2RAD});



//robot.sendArcSpeed(30, 5);

    while(1) {

//        data = robot.getOdomData();
//        cout<<"=================="<<endl
//            <<"X: "<<data.x<<endl
//            <<"Y: "<<data.y<<endl
//            <<"Fi: "<<data.fi<<endl;

        usleep(1000*1000*0.5);
    }

    return 0;
}