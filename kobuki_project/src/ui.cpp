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

    LaserMeasurement laserData = kobuki->lidarInterface.getLaserData();
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));

    /// V copyOfLaserData mame data z lidaru
    /// Call paintEvent
    updateEnviromentMap = true;
    kobuki->updateGlobalMap();
    kobuki->mapInterface.showMap();
    update();
}

void MainWindow::paintEvent(QPaintEvent *paintEvent)
{
    QPainter painter(this);
    ///prekreslujem lidar len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::white);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena

    QPen wall;
    wall.setStyle(Qt::SolidLine); //styl pera - plna ciara
    wall.setWidth(5);//hrubka pera -3pixely
    wall.setColor(Qt::black);//farba je zelena

    QRect rect;//(20,120,700,500);
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit



    painter.drawRect(rect);//vykreslite stvorec
    if(updateEnviromentMap)
    {
        syslog(LOG_DEBUG, "Update enviroment map");

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
    syslog(LOG_WARNING, "Function not implemented yet!");
};

void MainWindow::on_button_stop_mapping_clicked(){
    syslog(LOG_WARNING, "Function not implemented yet!");
};

void MainWindow::on_button_map_reset_clicked(){
    syslog(LOG_WARNING, "Function not implemented yet!");
};

void MainWindow::on_button_map_save_clicked(){
    syslog(LOG_WARNING, "Function not implemented yet!");
};

void MainWindow::on_button_map_load_clicked(){
    syslog(LOG_WARNING, "Function not implemented yet!");
};