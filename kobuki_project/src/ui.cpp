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
    timer->start(1000);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::refresh() {
    RobotPose odometry = robot->getOdomData();
    setOdometryGuiValues(odometry.x, odometry.y, odometry.fi);

    lidar->laserData_mtx.lock();
    LaserMeasurement laserData = lidar->getLaserData();
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));

    /// V copyOfLaserData mame data z lidaru
    lidar->laserData_mtx.unlock();
    paintEnviromentMap(nullptr);
}

void MainWindow::paintEnviromentMap(QPaintEvent *paintEvent)
{
    QPainter painter(this);
    ///prekreslujem lidar len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;//(20,120,700,500);
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit



    painter.drawRect(rect);//vykreslite stvorec
    if(updateEnviromentMap)
    {

        lidar->laserData_mtx.lock();//lock.. idem robit s premennou ktoru ine vlakno moze prepisovat...
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
        lidar->laserData_mtx.unlock();//unlock..skoncil som
    }
}

void MainWindow::on_pushButton_9_clicked() //start button
{

//    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
//    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
//    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);
/////toto je prepojenie signalu o zmene udajov, na signal
//    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));
}

void MainWindow::on_pushButton_2_clicked() //forward
{
//    //pohyb dopredu
//    std::vector<unsigned char> mess=robot.setTranslationSpeed(100);
//
/////ak by ste chceli miesto pohybu dopredu napriklad pohyb po kruznici s polomerom 1 meter zavolali by ste funkciu takto:
///// std::vector<unsigned char> mess=robot.setArcSpeed(100,1000);
//    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
//    {
//
//    }
}

void MainWindow::on_pushButton_3_clicked() //back
{
//    std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
//    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
//    {
//
//    }
}

void MainWindow::on_pushButton_6_clicked() //left
{
//
//    std::vector<unsigned char> mess=robot.setRotationSpeed(M_PI/2);
//    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
//    {
//
//    }
}

void MainWindow::on_pushButton_5_clicked()//right
{
//
//    std::vector<unsigned char> mess=robot.setRotationSpeed(-M_PI/2);
//    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
//    {
//
//    }
}

void MainWindow::on_pushButton_4_clicked() //stop
{
//    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
//    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
//    {
//
//    }
}

void MainWindow::setOdometryGuiValues(double robotX,double robotY,double robotFi)
{
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
}