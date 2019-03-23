//
// Created by jakub on 13.2.2019.
//
#include "../include/ui.h"
#include "../ui/ui_mainwindow.h"



MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    /// Call timer to refresh window values periodically

    auto *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(refresh()));
    timer->start(250);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::refresh() {
    RobotPose odometry = kobuki->robotInterface.getOdomData();
    setOdometryGuiValues(odometry.x, odometry.y, odometry.fi);

    /// V copyOfLaserData mame data z lidaru
    /// Call paintEvent
    updateEnviromentMap = true;

    LaserMeasurement laserData = kobuki->lidarInterface.getLaserData();
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));

    if(scanningEnviroment){
        kobuki->updateGlobalMap();
    }

    /// Output map contains only zeros and ones (0 -> space, 1 -> wall)
    enviromentMap = kobuki->mapInterface.getCVMatMap(1);

    update();
}

void MainWindow::paintEvent(QPaintEvent *paintEvent)
{
    QPainter painter(this);

    painter.setBrush(Qt::white); // background color

    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    QPen wall;
    wall.setStyle(Qt::SolidLine);
    wall.setColor(Qt::black);

    QPen grid;
    grid.setStyle(Qt::SolidLine);
    grid.setColor(Qt::gray);
    grid.setWidth(1);

    QRect rect;//(20,120,700,500);
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit

    painter.drawRect(rect);//vykreslite stvorec
    if(updateEnviromentMap)
    {
//        syslog(LOG_DEBUG, "Update enviroment map");

        paint_mux.lock();//lock.. idem robit s premennou ktoru ine vlakno moze prepisovat...
        updateEnviromentMap = false;

        painter.setPen(pero);
        ///teraz sa tu kreslia udaje z lidaru. ak chcete, prerobte
        for(int k=0;k<copyOfLaserData.numberOfScans;k++)
        {

            //tu sa rata z polarnych suradnic na kartezske, a zaroven sa upravuje mierka aby sme sa zmestili do
            //do vyhradeneho stvorca aspon castou merania.. ale nieje to pekne, krajsie by bolo
            //keby ste nastavovali mierku tak,aby bolo v okne zobrazene cele meranie (treba najst min a max pre x a y suradnicu a podla toho to prenasobit)
            int dist=copyOfLaserData.Data[k].scanDistance/15;//delim 15 aby som sa aspon niektorymi udajmi zmestil do okna.
            int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
            int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
            if(rect.contains(xp,yp))
                painter.drawEllipse(QPoint(xp, yp),2,2);//vykreslime kruh s polomerom 2px
        }

        /// Calculate scaling
        int wall_size = (int) MAX(rect.width() / enviromentMap.rows, rect.height() / enviromentMap.cols) + 1;
        wall.setWidth((wall_size < 1) ? 1 : wall_size);

        painter.setPen(wall);

        for(int i=0; i<enviromentMap.rows; i++){
            for(int j=0; j<enviromentMap.cols; j++){
                unsigned short p = kobuki->mapInterface.getPointValue({i,j});
                if(p >= 1){
                    painter.drawPoint(rect.topLeft().x() + (i * rect.width() / enviromentMap.rows), rect.topLeft().y() + (j * rect.height() / enviromentMap.cols));
                }
            }
        }

//        /// Paint grid
//        painter.setPen(grid);
//
//        for(int i = rect.top(); i < rect.bottom(); i += wall_size - 1){
//            painter.drawLine(QPoint(rect.left(), i),QPoint(rect.right(), i));
//        }
//        for(int j = rect.left(); j < rect.right(); j += wall_size - 1){
//            painter.drawLine(QPoint(j, rect.top()),QPoint(j, rect.bottom()));
//        }


        paint_mux.unlock();//unlock..skoncil som
    }
}

void MainWindow::setOdometryGuiValues(double robotX,double robotY,double robotFi)
{
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
}

/**************************
           SLOTS
 **************************/

void MainWindow::on_button_right_clicked(){
    kobuki->robotInterface.sendRotationSpeed(-1);
}

void MainWindow::on_button_left_clicked(){
    kobuki->robotInterface.sendRotationSpeed(1);
};

void MainWindow::on_button_forward_clicked(){
    kobuki->robotInterface.sendTranslationSpeed(250);
};

void MainWindow::on_button_back_clicked(){
    kobuki->robotInterface.sendTranslationSpeed(-250);
};

void MainWindow::on_button_stop_clicked(){
    kobuki->robotInterface.sendTranslationSpeed(0);
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
    kobuki->mapInterface.clearMap();
};

void MainWindow::on_button_map_save_clicked(){
    string file_name = ui->input_file_name->text().toUtf8().constData();
    syslog(LOG_INFO, "Saving map as: %s", file_name.c_str());
    kobuki->mapInterface.saveToFile(file_name);
};

void MainWindow::on_button_map_load_clicked(){
    string file_name = ui->input_file_name->text().toUtf8().constData();
    syslog(LOG_INFO, "Loading map from file: %s", file_name.c_str());
    kobuki->mapInterface = RobotMap(file_name);
};