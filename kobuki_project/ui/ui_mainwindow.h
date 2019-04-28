/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *upPanel_layout;
    QGroupBox *controls_box;
    QGridLayout *gridLayout;
    QPushButton *button_back;
    QPushButton *button_left;
    QPushButton *button_forward;
    QPushButton *button_stop;
    QPushButton *button_right;
    QSpacerItem *horizontalSpacer;
    QGroupBox *mapActions_box;
    QVBoxLayout *mapOperations;
    QPushButton *button_start_mapping;
    QPushButton *button_stop_mapping;
    QPushButton *button_map_reset;
    QPushButton *button_map_load;
    QPushButton *button_map_save;
    QHBoxLayout *downPanel_layout;
    QGroupBox *visualizer_box;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_4;
    QLabel *visualizer;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *visualizerOptions_box;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_3;
    QRadioButton *localPlanner_rb;
    QRadioButton *globalPlanner_rb;
    QCheckBox *show_env;
    QCheckBox *show_laser;
    QCheckBox *show_waypoins;
    QCheckBox *show_path;
    QCheckBox *show_floodfill;
    QCheckBox *show_robot;
    QGroupBox *goal_box;
    QGridLayout *goals;
    QLabel *label_6;
    QLineEdit *goalY;
    QLabel *label_5;
    QPushButton *btn_goToGoal;
    QLineEdit *goalX;
    QGroupBox *goalSpace_box;
    QGridLayout *gridLayout_2;
    QRadioButton *spaceRobot;
    QRadioButton *spaceOrigin;
    QLabel *goalStatus;
    QLabel *label_7;
    QGroupBox *resetodom_box;
    QGridLayout *resetOdom;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_9;
    QPushButton *btn_reset;
    QLineEdit *resetY;
    QLineEdit *resetX;
    QLineEdit *resetRot;
    QGroupBox *odom_box;
    QGridLayout *odom;
    QLabel *label_3;
    QLabel *label_2;
    QLineEdit *lineEdit_4;
    QLineEdit *lineEdit_3;
    QLineEdit *lineEdit_2;
    QLabel *label;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1033, 1000);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        upPanel_layout = new QHBoxLayout();
        upPanel_layout->setSpacing(6);
        upPanel_layout->setObjectName(QStringLiteral("upPanel_layout"));
        upPanel_layout->setSizeConstraint(QLayout::SetFixedSize);
        controls_box = new QGroupBox(centralWidget);
        controls_box->setObjectName(QStringLiteral("controls_box"));
        controls_box->setStyleSheet(QLatin1String("QGroupBox {\n"
"    font: bold;\n"
"    border: 1px solid silver;\n"
"    border-radius: 6px;\n"
"    margin-top: 6px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 7px;\n"
"    padding: 0px 5px 0px 5px;\n"
"}"));
        gridLayout = new QGridLayout(controls_box);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setSizeConstraint(QLayout::SetMinimumSize);
        button_back = new QPushButton(controls_box);
        button_back->setObjectName(QStringLiteral("button_back"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(button_back->sizePolicy().hasHeightForWidth());
        button_back->setSizePolicy(sizePolicy);

        gridLayout->addWidget(button_back, 2, 1, 1, 1);

        button_left = new QPushButton(controls_box);
        button_left->setObjectName(QStringLiteral("button_left"));
        sizePolicy.setHeightForWidth(button_left->sizePolicy().hasHeightForWidth());
        button_left->setSizePolicy(sizePolicy);

        gridLayout->addWidget(button_left, 1, 0, 1, 1);

        button_forward = new QPushButton(controls_box);
        button_forward->setObjectName(QStringLiteral("button_forward"));
        sizePolicy.setHeightForWidth(button_forward->sizePolicy().hasHeightForWidth());
        button_forward->setSizePolicy(sizePolicy);

        gridLayout->addWidget(button_forward, 0, 1, 1, 1);

        button_stop = new QPushButton(controls_box);
        button_stop->setObjectName(QStringLiteral("button_stop"));
        sizePolicy.setHeightForWidth(button_stop->sizePolicy().hasHeightForWidth());
        button_stop->setSizePolicy(sizePolicy);

        gridLayout->addWidget(button_stop, 1, 1, 1, 1);

        button_right = new QPushButton(controls_box);
        button_right->setObjectName(QStringLiteral("button_right"));
        sizePolicy.setHeightForWidth(button_right->sizePolicy().hasHeightForWidth());
        button_right->setSizePolicy(sizePolicy);

        gridLayout->addWidget(button_right, 1, 2, 1, 1);


        upPanel_layout->addWidget(controls_box);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        upPanel_layout->addItem(horizontalSpacer);

        mapActions_box = new QGroupBox(centralWidget);
        mapActions_box->setObjectName(QStringLiteral("mapActions_box"));
        mapActions_box->setStyleSheet(QLatin1String("QGroupBox {\n"
"    font: bold;\n"
"    border: 1px solid silver;\n"
"    border-radius: 6px;\n"
"    margin-top: 6px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 7px;\n"
"    padding: 0px 5px 0px 5px;\n"
"}"));
        mapOperations = new QVBoxLayout(mapActions_box);
        mapOperations->setSpacing(6);
        mapOperations->setContentsMargins(11, 11, 11, 11);
        mapOperations->setObjectName(QStringLiteral("mapOperations"));
        button_start_mapping = new QPushButton(mapActions_box);
        button_start_mapping->setObjectName(QStringLiteral("button_start_mapping"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(button_start_mapping->sizePolicy().hasHeightForWidth());
        button_start_mapping->setSizePolicy(sizePolicy1);
        button_start_mapping->setMinimumSize(QSize(162, 0));

        mapOperations->addWidget(button_start_mapping);

        button_stop_mapping = new QPushButton(mapActions_box);
        button_stop_mapping->setObjectName(QStringLiteral("button_stop_mapping"));
        sizePolicy1.setHeightForWidth(button_stop_mapping->sizePolicy().hasHeightForWidth());
        button_stop_mapping->setSizePolicy(sizePolicy1);
        button_stop_mapping->setMinimumSize(QSize(162, 0));

        mapOperations->addWidget(button_stop_mapping);

        button_map_reset = new QPushButton(mapActions_box);
        button_map_reset->setObjectName(QStringLiteral("button_map_reset"));
        sizePolicy1.setHeightForWidth(button_map_reset->sizePolicy().hasHeightForWidth());
        button_map_reset->setSizePolicy(sizePolicy1);
        button_map_reset->setMinimumSize(QSize(162, 0));

        mapOperations->addWidget(button_map_reset);

        button_map_load = new QPushButton(mapActions_box);
        button_map_load->setObjectName(QStringLiteral("button_map_load"));

        mapOperations->addWidget(button_map_load);

        button_map_save = new QPushButton(mapActions_box);
        button_map_save->setObjectName(QStringLiteral("button_map_save"));
        sizePolicy1.setHeightForWidth(button_map_save->sizePolicy().hasHeightForWidth());
        button_map_save->setSizePolicy(sizePolicy1);
        button_map_save->setMinimumSize(QSize(162, 0));

        mapOperations->addWidget(button_map_save);


        upPanel_layout->addWidget(mapActions_box);


        verticalLayout->addLayout(upPanel_layout);

        downPanel_layout = new QHBoxLayout();
        downPanel_layout->setSpacing(6);
        downPanel_layout->setObjectName(QStringLiteral("downPanel_layout"));
        downPanel_layout->setSizeConstraint(QLayout::SetFixedSize);
        visualizer_box = new QGroupBox(centralWidget);
        visualizer_box->setObjectName(QStringLiteral("visualizer_box"));
        visualizer_box->setStyleSheet(QLatin1String("QGroupBox {\n"
"    font: bold;\n"
"    border: 1px solid silver;\n"
"    border-radius: 6px;\n"
"    margin-top: 6px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 7px;\n"
"    padding: 0px 5px 0px 5px;\n"
"}"));
        verticalLayout_3 = new QVBoxLayout(visualizer_box);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        visualizer = new QLabel(visualizer_box);
        visualizer->setObjectName(QStringLiteral("visualizer"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Ignored);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(visualizer->sizePolicy().hasHeightForWidth());
        visualizer->setSizePolicy(sizePolicy2);
        visualizer->setFrameShape(QFrame::NoFrame);

        verticalLayout_4->addWidget(visualizer);


        verticalLayout_3->addLayout(verticalLayout_4);


        downPanel_layout->addWidget(visualizer_box);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetMinAndMaxSize);
        verticalLayout_2->setContentsMargins(-1, 0, -1, -1);
        visualizerOptions_box = new QGroupBox(centralWidget);
        visualizerOptions_box->setObjectName(QStringLiteral("visualizerOptions_box"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(visualizerOptions_box->sizePolicy().hasHeightForWidth());
        visualizerOptions_box->setSizePolicy(sizePolicy3);
        visualizerOptions_box->setStyleSheet(QLatin1String("QGroupBox {\n"
"    font: bold;\n"
"    border: 1px solid silver;\n"
"    border-radius: 6px;\n"
"    margin-top: 6px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 7px;\n"
"    padding: 0px 5px 0px 5px;\n"
"}"));
        verticalLayout_5 = new QVBoxLayout(visualizerOptions_box);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        groupBox = new QGroupBox(visualizerOptions_box);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        sizePolicy3.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy3);
        groupBox->setStyleSheet(QStringLiteral("font: 57 italic 11pt \"Ubuntu\";"));
        gridLayout_3 = new QGridLayout(groupBox);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        localPlanner_rb = new QRadioButton(groupBox);
        localPlanner_rb->setObjectName(QStringLiteral("localPlanner_rb"));

        gridLayout_3->addWidget(localPlanner_rb, 0, 0, 1, 1);

        globalPlanner_rb = new QRadioButton(groupBox);
        globalPlanner_rb->setObjectName(QStringLiteral("globalPlanner_rb"));
        globalPlanner_rb->setChecked(true);

        gridLayout_3->addWidget(globalPlanner_rb, 1, 0, 1, 1);


        verticalLayout_5->addWidget(groupBox);

        show_env = new QCheckBox(visualizerOptions_box);
        show_env->setObjectName(QStringLiteral("show_env"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(show_env->sizePolicy().hasHeightForWidth());
        show_env->setSizePolicy(sizePolicy4);
        show_env->setChecked(true);

        verticalLayout_5->addWidget(show_env);

        show_laser = new QCheckBox(visualizerOptions_box);
        show_laser->setObjectName(QStringLiteral("show_laser"));
        sizePolicy4.setHeightForWidth(show_laser->sizePolicy().hasHeightForWidth());
        show_laser->setSizePolicy(sizePolicy4);
        show_laser->setChecked(true);

        verticalLayout_5->addWidget(show_laser);

        show_waypoins = new QCheckBox(visualizerOptions_box);
        show_waypoins->setObjectName(QStringLiteral("show_waypoins"));
        sizePolicy4.setHeightForWidth(show_waypoins->sizePolicy().hasHeightForWidth());
        show_waypoins->setSizePolicy(sizePolicy4);
        show_waypoins->setChecked(true);

        verticalLayout_5->addWidget(show_waypoins);

        show_path = new QCheckBox(visualizerOptions_box);
        show_path->setObjectName(QStringLiteral("show_path"));
        sizePolicy4.setHeightForWidth(show_path->sizePolicy().hasHeightForWidth());
        show_path->setSizePolicy(sizePolicy4);
        show_path->setChecked(true);

        verticalLayout_5->addWidget(show_path);

        show_floodfill = new QCheckBox(visualizerOptions_box);
        show_floodfill->setObjectName(QStringLiteral("show_floodfill"));
        sizePolicy4.setHeightForWidth(show_floodfill->sizePolicy().hasHeightForWidth());
        show_floodfill->setSizePolicy(sizePolicy4);

        verticalLayout_5->addWidget(show_floodfill);

        show_robot = new QCheckBox(visualizerOptions_box);
        show_robot->setObjectName(QStringLiteral("show_robot"));
        sizePolicy4.setHeightForWidth(show_robot->sizePolicy().hasHeightForWidth());
        show_robot->setSizePolicy(sizePolicy4);
        show_robot->setChecked(true);

        verticalLayout_5->addWidget(show_robot);


        verticalLayout_2->addWidget(visualizerOptions_box);

        goal_box = new QGroupBox(centralWidget);
        goal_box->setObjectName(QStringLiteral("goal_box"));
        goal_box->setStyleSheet(QLatin1String("QGroupBox {\n"
"    font: bold;\n"
"    border: 1px solid silver;\n"
"    border-radius: 6px;\n"
"    margin-top: 6px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 7px;\n"
"    padding: 0px 5px 0px 5px;\n"
"}"));
        goals = new QGridLayout(goal_box);
        goals->setSpacing(6);
        goals->setContentsMargins(11, 11, 11, 11);
        goals->setObjectName(QStringLiteral("goals"));
        label_6 = new QLabel(goal_box);
        label_6->setObjectName(QStringLiteral("label_6"));

        goals->addWidget(label_6, 2, 0, 1, 1);

        goalY = new QLineEdit(goal_box);
        goalY->setObjectName(QStringLiteral("goalY"));
        sizePolicy4.setHeightForWidth(goalY->sizePolicy().hasHeightForWidth());
        goalY->setSizePolicy(sizePolicy4);

        goals->addWidget(goalY, 3, 1, 1, 1);

        label_5 = new QLabel(goal_box);
        label_5->setObjectName(QStringLiteral("label_5"));

        goals->addWidget(label_5, 3, 0, 1, 1);

        btn_goToGoal = new QPushButton(goal_box);
        btn_goToGoal->setObjectName(QStringLiteral("btn_goToGoal"));

        goals->addWidget(btn_goToGoal, 5, 1, 1, 1);

        goalX = new QLineEdit(goal_box);
        goalX->setObjectName(QStringLiteral("goalX"));
        sizePolicy4.setHeightForWidth(goalX->sizePolicy().hasHeightForWidth());
        goalX->setSizePolicy(sizePolicy4);

        goals->addWidget(goalX, 2, 1, 1, 1);

        goalSpace_box = new QGroupBox(goal_box);
        goalSpace_box->setObjectName(QStringLiteral("goalSpace_box"));
        goalSpace_box->setStyleSheet(QLatin1String("\n"
"font: 57 italic 11pt \"Ubuntu\";"));
        gridLayout_2 = new QGridLayout(goalSpace_box);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        spaceRobot = new QRadioButton(goalSpace_box);
        spaceRobot->setObjectName(QStringLiteral("spaceRobot"));
        spaceRobot->setChecked(true);

        gridLayout_2->addWidget(spaceRobot, 0, 0, 1, 1);

        spaceOrigin = new QRadioButton(goalSpace_box);
        spaceOrigin->setObjectName(QStringLiteral("spaceOrigin"));

        gridLayout_2->addWidget(spaceOrigin, 1, 0, 1, 1);


        goals->addWidget(goalSpace_box, 1, 0, 1, 2);

        goalStatus = new QLabel(goal_box);
        goalStatus->setObjectName(QStringLiteral("goalStatus"));
        goalStatus->setStyleSheet(QLatin1String("QLabel {\n"
"color:  green;\n"
"}"));

        goals->addWidget(goalStatus, 6, 1, 1, 1);

        label_7 = new QLabel(goal_box);
        label_7->setObjectName(QStringLiteral("label_7"));

        goals->addWidget(label_7, 6, 0, 1, 1);


        verticalLayout_2->addWidget(goal_box);

        resetodom_box = new QGroupBox(centralWidget);
        resetodom_box->setObjectName(QStringLiteral("resetodom_box"));
        resetodom_box->setStyleSheet(QLatin1String("QGroupBox {\n"
"    font: bold;\n"
"    border: 1px solid silver;\n"
"    border-radius: 6px;\n"
"    margin-top: 6px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 7px;\n"
"    padding: 0px 5px 0px 5px;\n"
"}"));
        resetOdom = new QGridLayout(resetodom_box);
        resetOdom->setSpacing(6);
        resetOdom->setContentsMargins(11, 11, 11, 11);
        resetOdom->setObjectName(QStringLiteral("resetOdom"));
        label_10 = new QLabel(resetodom_box);
        label_10->setObjectName(QStringLiteral("label_10"));

        resetOdom->addWidget(label_10, 2, 0, 1, 1);

        label_11 = new QLabel(resetodom_box);
        label_11->setObjectName(QStringLiteral("label_11"));

        resetOdom->addWidget(label_11, 0, 0, 1, 1);

        label_9 = new QLabel(resetodom_box);
        label_9->setObjectName(QStringLiteral("label_9"));

        resetOdom->addWidget(label_9, 3, 0, 1, 1);

        btn_reset = new QPushButton(resetodom_box);
        btn_reset->setObjectName(QStringLiteral("btn_reset"));

        resetOdom->addWidget(btn_reset, 4, 1, 1, 1);

        resetY = new QLineEdit(resetodom_box);
        resetY->setObjectName(QStringLiteral("resetY"));
        sizePolicy4.setHeightForWidth(resetY->sizePolicy().hasHeightForWidth());
        resetY->setSizePolicy(sizePolicy4);

        resetOdom->addWidget(resetY, 2, 1, 1, 1);

        resetX = new QLineEdit(resetodom_box);
        resetX->setObjectName(QStringLiteral("resetX"));
        sizePolicy4.setHeightForWidth(resetX->sizePolicy().hasHeightForWidth());
        resetX->setSizePolicy(sizePolicy4);

        resetOdom->addWidget(resetX, 0, 1, 1, 1);

        resetRot = new QLineEdit(resetodom_box);
        resetRot->setObjectName(QStringLiteral("resetRot"));
        sizePolicy4.setHeightForWidth(resetRot->sizePolicy().hasHeightForWidth());
        resetRot->setSizePolicy(sizePolicy4);

        resetOdom->addWidget(resetRot, 3, 1, 1, 1);


        verticalLayout_2->addWidget(resetodom_box);

        odom_box = new QGroupBox(centralWidget);
        odom_box->setObjectName(QStringLiteral("odom_box"));
        odom_box->setStyleSheet(QLatin1String("QGroupBox {\n"
"    font: bold;\n"
"    border: 1px solid silver;\n"
"    border-radius: 6px;\n"
"    margin-top: 6px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 7px;\n"
"    padding: 0px 5px 0px 5px;\n"
"}"));
        odom = new QGridLayout(odom_box);
        odom->setSpacing(6);
        odom->setContentsMargins(11, 11, 11, 11);
        odom->setObjectName(QStringLiteral("odom"));
        label_3 = new QLabel(odom_box);
        label_3->setObjectName(QStringLiteral("label_3"));

        odom->addWidget(label_3, 2, 0, 1, 1);

        label_2 = new QLabel(odom_box);
        label_2->setObjectName(QStringLiteral("label_2"));

        odom->addWidget(label_2, 1, 0, 1, 1);

        lineEdit_4 = new QLineEdit(odom_box);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));
        sizePolicy4.setHeightForWidth(lineEdit_4->sizePolicy().hasHeightForWidth());
        lineEdit_4->setSizePolicy(sizePolicy4);
        lineEdit_4->setFocusPolicy(Qt::NoFocus);
        lineEdit_4->setReadOnly(true);

        odom->addWidget(lineEdit_4, 2, 1, 1, 1);

        lineEdit_3 = new QLineEdit(odom_box);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));
        sizePolicy4.setHeightForWidth(lineEdit_3->sizePolicy().hasHeightForWidth());
        lineEdit_3->setSizePolicy(sizePolicy4);
        lineEdit_3->setFocusPolicy(Qt::NoFocus);
        lineEdit_3->setReadOnly(true);

        odom->addWidget(lineEdit_3, 1, 1, 1, 1);

        lineEdit_2 = new QLineEdit(odom_box);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));
        sizePolicy4.setHeightForWidth(lineEdit_2->sizePolicy().hasHeightForWidth());
        lineEdit_2->setSizePolicy(sizePolicy4);
        lineEdit_2->setFocusPolicy(Qt::NoFocus);
        lineEdit_2->setReadOnly(true);

        odom->addWidget(lineEdit_2, 0, 1, 1, 1);

        label = new QLabel(odom_box);
        label->setObjectName(QStringLiteral("label"));

        odom->addWidget(label, 0, 0, 1, 1);


        verticalLayout_2->addWidget(odom_box);


        downPanel_layout->addLayout(verticalLayout_2);


        verticalLayout->addLayout(downPanel_layout);

        MainWindow->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        controls_box->setTitle(QApplication::translate("MainWindow", "Controls", Q_NULLPTR));
        button_back->setText(QApplication::translate("MainWindow", "Back", Q_NULLPTR));
        button_left->setText(QApplication::translate("MainWindow", "Left", Q_NULLPTR));
        button_forward->setText(QApplication::translate("MainWindow", "Forward", Q_NULLPTR));
        button_stop->setText(QApplication::translate("MainWindow", "Stop", Q_NULLPTR));
        button_right->setText(QApplication::translate("MainWindow", "Right", Q_NULLPTR));
        mapActions_box->setTitle(QApplication::translate("MainWindow", "Map actions", Q_NULLPTR));
        button_start_mapping->setText(QApplication::translate("MainWindow", "Start mapping", Q_NULLPTR));
        button_stop_mapping->setText(QApplication::translate("MainWindow", "Stop mapping", Q_NULLPTR));
        button_map_reset->setText(QApplication::translate("MainWindow", "Clear map", Q_NULLPTR));
        button_map_load->setText(QApplication::translate("MainWindow", "Load map", Q_NULLPTR));
        button_map_save->setText(QApplication::translate("MainWindow", "Save map", Q_NULLPTR));
        visualizer_box->setTitle(QApplication::translate("MainWindow", "Visualizer", Q_NULLPTR));
        visualizer->setText(QApplication::translate("MainWindow", "TextLabel", Q_NULLPTR));
        visualizerOptions_box->setTitle(QApplication::translate("MainWindow", "Visualizer options", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("MainWindow", "Planner", Q_NULLPTR));
        localPlanner_rb->setText(QApplication::translate("MainWindow", "Local planner", Q_NULLPTR));
        globalPlanner_rb->setText(QApplication::translate("MainWindow", "Global planner", Q_NULLPTR));
        show_env->setText(QApplication::translate("MainWindow", "environment", Q_NULLPTR));
        show_laser->setText(QApplication::translate("MainWindow", "laser scan", Q_NULLPTR));
        show_waypoins->setText(QApplication::translate("MainWindow", "waypoints", Q_NULLPTR));
        show_path->setText(QApplication::translate("MainWindow", "path", Q_NULLPTR));
        show_floodfill->setText(QApplication::translate("MainWindow", "flood fill", Q_NULLPTR));
        show_robot->setText(QApplication::translate("MainWindow", "robot", Q_NULLPTR));
        goal_box->setTitle(QApplication::translate("MainWindow", "Goal", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "X:", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "Y:", Q_NULLPTR));
        btn_goToGoal->setText(QApplication::translate("MainWindow", "GO", Q_NULLPTR));
        goalSpace_box->setTitle(QApplication::translate("MainWindow", "Space", Q_NULLPTR));
        spaceRobot->setText(QApplication::translate("MainWindow", "Robot", Q_NULLPTR));
        spaceOrigin->setText(QApplication::translate("MainWindow", "Origin", Q_NULLPTR));
        goalStatus->setText(QApplication::translate("MainWindow", "No goal", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "Status", Q_NULLPTR));
        resetodom_box->setTitle(QApplication::translate("MainWindow", "Reset odometry", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "Y:", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "X:", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "Rot:", Q_NULLPTR));
        btn_reset->setText(QApplication::translate("MainWindow", "RESET", Q_NULLPTR));
        odom_box->setTitle(QApplication::translate("MainWindow", "Odometry", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Rot:", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "Y:", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "X:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
