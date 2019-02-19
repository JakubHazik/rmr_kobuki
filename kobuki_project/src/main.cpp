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

    robot.sendTranslationSpeed(10);

    while(1) {
        auto data = robot.getOdomData();
        cout<<"=================="<<endl
            <<"X: "<<data.x<<endl
            <<"Y: "<<data.y<<endl
            <<"Fi: "<<data.fi<<endl;
        usleep(1000*500);
    }

    return 0;
}