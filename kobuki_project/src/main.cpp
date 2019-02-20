//
// Created by jakub on 13.2.2019.
//

#include "include/ui.h"
#include "include/robot_interface.h"
#include <QApplication>


using namespace std;

int main(int argc, char *argv[]) {
//    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();
//
//    return a.exec();

    RobotInterface robot;

    usleep(1000*1000*3);

//    robot.sendTranslationSpeed(10);

    auto data = robot.getOdomData();

    while(1) {
//        robot.sendTranslationSpeed(40);
//        robot.sendRotationSpeed(1);

        data = robot.getOdomData();
        cout<<"=================="<<endl
            <<"X: "<<data.x<<endl
            <<"Y: "<<data.y<<endl
            <<"Fi: "<<data.fi<<endl
            <<"G: "<<robot.forOdomUseGyro<<endl;

        usleep(1000*1000*0.5);
    }

    return 0;
}