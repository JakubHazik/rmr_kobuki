//
// Created by jakub on 13.2.2019.
//

#include "include/ui.h"
#include <QApplication>


using namespace std;

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}