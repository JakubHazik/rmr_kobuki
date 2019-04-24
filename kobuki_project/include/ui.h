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
#include <future>
#include <chrono>

#include "kobuki.h"
#include "config_defines.h"


#define GO_BUTTON_TEXT "GO"
#define BREAK_BUTTON_TEXT "BREAK"

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
    void on_button_start_mapping_clicked();
    void on_button_stop_mapping_clicked();
    void on_button_map_reset_clicked();
    void on_button_map_save_clicked();
    void on_button_map_load_clicked();
    void on_btn_goToGoal_clicked();
    void on_btn_reset_clicked();

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

    void movementProcessing(bool processing);
    void movementIsDone();
    future<void> movementDone;

public slots:
    void setOdometryGuiValues(double robotX, double robotY, double robotFi);
    void refresh();
};

#endif //KOBUKI_PROJECT_UI_H
