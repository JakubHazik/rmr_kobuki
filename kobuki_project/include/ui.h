//
// Created by jakub on 13.2.2019.
//

#ifndef KOBUKI_PROJECT_UI_H
#define KOBUKI_PROJECT_UI_H

#include <QtWidgets>
#include <QMainWindow>
#include <QPushButton>
#include <QFileDialog>

#include <include/robot_interface.h>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();

        void setRobotInterface(RobotInterface *_robot){
            this->robot = _robot;
        }

    private slots:
        void on_pushButton_9_clicked();

        void on_pushButton_2_clicked();

        void on_pushButton_3_clicked();

        void on_pushButton_6_clicked();

        void on_pushButton_5_clicked();

        void on_pushButton_4_clicked();

    private:
        Ui::MainWindow *ui;

        RobotInterface *robot;

        LaserMeasurement copyOfLaserData;
        bool updateEnviromentMap;

        void paintEnviromentMap(QPaintEvent *paintEvent);

    public slots:
        void setOdometryGuiValues(double robotX,double robotY,double robotFi);

    signals:
        void odometryGuiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo

};

#endif //KOBUKI_PROJECT_UI_H
