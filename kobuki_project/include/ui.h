//
// Created by jakub on 13.2.2019.
//

#ifndef KOBUKI_PROJECT_UI_H
#define KOBUKI_PROJECT_UI_H

#include <QtWidgets>
#include <QMainWindow>
#include <QPushButton>
#include <QFileDialog>
#include <QTimer>

#include <opencv2/opencv.hpp>
#include "kobuki.h"
#include "config_defines.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    explicit MainWindow(Kobuki *kobuki, QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_button_right_clicked();
    void on_button_left_clicked();
    void on_button_forward_clicked();
    void on_button_back_clicked();
    void on_button_stop_clicked();

    void on_button_start_mapping_clicked();
    void on_button_stop_mapping_clicked();
    void on_button_map_reset_clicked();
    void on_button_map_save_clicked();
    void on_button_map_load_clicked();
    void on_button_go_to_pos_clicked();

private:

    Ui::MainWindow *ui;
    mutex paint_mux;
    QTimer timer;
    Kobuki *kobuki;
    LaserMeasurement copyOfLaserData;
    bool updateEnviromentMap = false;

    bool scanningEnviroment = false;

//    void paintEvent(QPaintEvent *paintEvent) {
//        QWidget::paintEvent(paintEvent);
//    }

public slots:
    void setOdometryGuiValues(double robotX, double robotY, double robotFi);
    void refresh();
};

#endif //KOBUKI_PROJECT_UI_H
