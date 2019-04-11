//
// Created by jakub on 13.2.2019.
//
#include "../include/ui.h"
#include "../ui/ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // Call timer to refresh window values periodically
    connect(&timer, SIGNAL(timeout()), this, SLOT(refresh()));
    timer.start(Kconfig::Ui::CANVAS_REFRESH_RATE);
}

MainWindow::MainWindow(Kobuki *kobuki, QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)  {
    ui->setupUi(this);
    this->kobuki = kobuki;

    // Call timer to refresh window values periodically
    connect(&timer, SIGNAL(timeout()), this, SLOT(refresh()));
    timer.start(Kconfig::Ui::CANVAS_REFRESH_RATE);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::refresh() {
//    syslog(LOG_INFO, "R");

    if (movementDone.valid()) {
        if (future_status::ready == movementDone.wait_for(std::chrono::seconds(0))) {
            movementIsDone();
        }
    }

    RobotPose odometry = kobuki->getRobotPosition();
    setOdometryGuiValues(odometry.x, odometry.y, odometry.fi);

    cv::Mat mat = kobuki->getEnvironmentAsImage(ui->show_env->isChecked(), ui->show_waypoins->isChecked(),
            ui->show_path->isChecked(), ui->show_floodfill->isChecked(), ui->show_laser->isChecked());
    QPixmap pixmap = QPixmap::fromImage(QImage((unsigned char*) mat.data, mat.cols, mat.rows, QImage::Format_RGB888));
    ui->visualizer->setPixmap(pixmap.scaled(ui->visualizer->width(),ui->visualizer->height(),Qt::KeepAspectRatio));

    update();
}

//void MainWindow::paintEvent(QPaintEvent *paintEvent)
//{
//    QPainter painter(this);
//
//    painter.setBrush(Qt::white); // background color
//
//    QPen pero;
//    pero.setStyle(Qt::SolidLine);
//    pero.setWidth(3);
//    pero.setColor(Qt::green);
//
//    QPen wall;
//    wall.setStyle(Qt::SolidLine);
//    wall.setColor(Qt::black);
//
//    QPen grid;
//    grid.setStyle(Qt::SolidLine);
//    grid.setColor(Qt::gray);
//    grid.setWidth(1);
//
//    QPen rob;
//    grid.setStyle(Qt::SolidLine);
//    grid.setColor(Qt::red);
//    grid.setWidth(10);
//
//    QRect rect;//(20,120,700,500);
//    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
//
//    painter.drawRect(rect);//vykreslite stvorec
//    if(updateEnviromentMap)
//    {
////        syslog(LOG_DEBUG, "Update enviroment map");
//
//        paint_mux.lock();//lock.. idem robit s premennou ktoru ine vlakno moze prepisovat...
//        updateEnviromentMap = false;
//
//        painter.setPen(pero);
//        ///teraz sa tu kreslia udaje z lidaru. ak chcete, prerobte
//        for(int k=0;k<copyOfLaserData.numberOfScans;k++)
//        {
//
//
//            int dist = (int) (copyOfLaserData.Data[k].scanDistance / kobuki->map.getResolution());
//            int xp = (int) (rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x());
//            int yp = (int) (rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y());
////            if(rect.contains(xp,yp))
////                painter.drawEllipse(QPoint(xp, yp),2,2);//vykreslime kruh s polomerom 2px
//        }
//
//        /// Calculate scaling
//        int wall_size = (int) MAX(rect.width() / enviromentMap.rows, rect.height() / enviromentMap.cols) + 1;
//        wall.setWidth((wall_size < 1) ? 1 : wall_size);
//
//        painter.setPen(wall);
//
//        for(int i=0; i<enviromentMap.rows; i++){
//            for(int j=0; j<enviromentMap.cols; j++){
//                unsigned short p = kobuki->map.getPointValue({i,j});
//                if(p >= 1){
//                    painter.drawPoint(rect.topLeft().x() + (i * rect.width() / enviromentMap.rows), rect.topLeft().y() + (j * rect.height() / enviromentMap.cols));
//                }
//            }
//        }
//
//        painter.setPen(rob);
//        RobotPose pos = kobuki->robotInterface->getOdomData();
//        painter.drawPoint((int) pos.x, (int) pos.y);
//
////        /// Paint grid
////        painter.setPen(grid);
////
////        for(int i = rect.top(); i < rect.bottom(); i += wall_size - 1){
////            painter.drawLine(QPoint(rect.left(), i),QPoint(rect.right(), i));
////        }
////        for(int j = rect.left(); j < rect.right(); j += wall_size - 1){
////            painter.drawLine(QPoint(j, rect.top()),QPoint(j, rect.bottom()));
////        }
//
//
//        paint_mux.unlock();//unlock..skoncil som
//    }
//}

void MainWindow::setOdometryGuiValues(double robotX,double robotY,double robotFi) {
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi*RAD2DEG));
}

/**************************
           SLOTS
 **************************/

void MainWindow::on_button_right_clicked(){
    syslog(LOG_INFO, "Move robot right");
//    kobuki->robotInterface->sendRotationSpeed(-1);
}

void MainWindow::on_button_left_clicked(){
    syslog(LOG_INFO, "Move robot left");
//    kobuki->robotInterface->sendRotationSpeed(1);
};

void MainWindow::on_button_forward_clicked(){
    syslog(LOG_INFO, "Move robot forward");
//    kobuki->robotInterface->sendTranslationSpeed(100);
};

void MainWindow::on_button_back_clicked(){
    syslog(LOG_INFO, "Move robot back");
//    kobuki->robotInterface->sendTranslationSpeed(-100);
};

void MainWindow::on_button_stop_clicked(){
    syslog(LOG_INFO, "Stop robot");
//    kobuki->robotInterface->sendTranslationSpeed(0);
    movementProcessing(true);
}

void MainWindow::on_button_start_mapping_clicked(){
    syslog(LOG_INFO, "Started mapping process");
    scanningEnviroment = true;
};

void MainWindow::on_button_stop_mapping_clicked(){
    syslog(LOG_INFO, "Stopping mapping process");
    scanningEnviroment = false;
};

void MainWindow::on_button_map_reset_clicked(){
    syslog(LOG_INFO, "Clearing enviroment map");
    kobuki->clearMap();
};

void MainWindow::on_button_map_save_clicked(){
    auto filename = QFileDialog::getSaveFileName(this, "Save map", QDir::currentPath());

    if (!filename.isEmpty()) {

        if (!filename.endsWith(Kconfig::Defaults::OPENCV_MAP_EXTENSION.c_str())) {
            filename += '.';
            filename += Kconfig::Defaults::OPENCV_MAP_EXTENSION.c_str();
        }

        kobuki->saveMapToFile(filename.toStdString());
    }
};

void MainWindow::on_button_map_load_clicked(){
//    QFileDialog fileDialog;
    auto filename = QFileDialog::getOpenFileName(this, "Load map", QDir::currentPath());

    if (!filename.isEmpty()) {
        auto a = filename.toStdString();
        kobuki->loadMapFromFile(filename.toStdString());
    }
};

void MainWindow::on_btn_goToGoal_clicked(){
    double x_to_go = ui->goalX->text().toDouble();
    double y_to_go = ui->goalY->text().toDouble();
    syslog(LOG_INFO, "Going x = %lf, y = %lf", x_to_go, y_to_go);

    SPACE space;
    if (ui->spaceOrigin->isChecked()) {
        space = SPACE::ORIGIN_SPACE;
    } else if (ui->spaceRobot->isChecked()) {
        space = SPACE::ROBOT_SPACE;
    } else {
        throw invalid_argument("no radio button is checked");
    }

    movementDone = std::async(std::launch::async, &Kobuki::sendRobotToPosition, kobuki, x_to_go, y_to_go, space);

    ui->goalStatus->setText("Processing");
    ui->goalStatus->setStyleSheet("QLabel { color : red }");

    movementProcessing(true);
}

void MainWindow::movementProcessing(bool processing) {
    ui->mapActions_box->setEnabled(!processing);
//    ui->button_map_save->setEnabled(true);
    ui->controls_box->setEnabled(!processing);
    ui->goal_box->setEnabled(!processing);
    ui->resetodom_box->setEnabled(!processing);
}

void MainWindow::on_btn_reset_clicked() {
    auto x = ui->resetX->text().toDouble();
    auto y = ui->resetY->text().toDouble();
    auto fi = ui->resetRot->text().toDouble() * DEG2RAD;
    kobuki->setRobotActualPosition(x, y, fi);
}

void MainWindow::movementIsDone() {
    movementProcessing(false);
    ui->goalStatus->setText("Movement Done");
    ui->goalStatus->setStyleSheet("QLabel { color : green }");
}

