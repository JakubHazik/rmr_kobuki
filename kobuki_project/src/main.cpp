//
// Created by jakub on 13.2.2019.
//

#include "include/ui.h"
#include "include/robot_interface.h"
#include <QApplication>
#include <unistd.h> // for getopt


using namespace std;

int main(int argc, char *argv[]) {
    /// Parse starting arguments
    int opt;
    bool debug = false, gui = false;

    while ((opt = getopt(argc, argv, "dg")) != -1) {
        switch (opt) {
            case 'd':
                debug = true;
                break;
            case 'g':
                gui = true;
                break;
            default: /* '?' */
                fprintf(stderr, "Usage: %s [-d] <debug messages> [-g] <run with gui>\n",
                        argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    openlog("kobuki", LOG_PERROR, LOG_DAEMON);
    if(debug){
        // Log debug messages to journal as well
        setlogmask (LOG_UPTO (LOG_DEBUG));
        syslog(LOG_INFO, "Debug messages are on. Make sure journal level is set to debug as well.\n"
                         "\tSee /etc/systemd/journald.conf for your settings.");
    }

    RobotInterface robot;
    usleep(1000*1000*2);

    if(gui){
        QApplication a(argc, argv);
        MainWindow w;
        w.setRobotInterface(&robot);
        w.show();

        syslog(LOG_INFO, "Kobuki started in graphical mode!");

        return a.exec();
    } else {
        syslog(LOG_INFO, "Kobuki started in conosle mode!");

        /// Endless loop!
        while(1){};
    }


//    auto data = robot.getOdomData();
//
//
//    robot.addCommandToQueue({data.x + 1000, data.y + 0, 0 * DEG2RAD});
    //robot.addCommandToQueue({data.x + 2000, data.y + 0, 0 * DEG2RAD});

//    while(1) {
//
//        data = robot.getOdomData();
//        cout<<"=================="<<endl
//            <<"X: "<<data.x<<endl
//            <<"Y: "<<data.y<<endl
//            <<"Fi: "<<data.fi<<endl
//            <<"G: "<<robot.forOdomUseGyro<<endl;
//
//        usleep(1000*1000*0.5);
//    }

    return 0;
}