//
// Created by jakub on 13.2.2019.
//

#include <QApplication>
#include <unistd.h> // for getopt
#include <syslog.h>

#include <include/kobuki.h>
#include <include/ui.h>

using namespace std;

int main(int argc, char *argv[]) {
    /// Parse starting arguments
    int opt;
    bool debug = false, gui = true;

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

    Kobuki kobuki;

    if(gui){
        QApplication a(argc, argv);
        MainWindow w(&kobuki);
        w.show();

        syslog(LOG_INFO, "Kobuki started in graphical mode!");

        return a.exec();
    }
}