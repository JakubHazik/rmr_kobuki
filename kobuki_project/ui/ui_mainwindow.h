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
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_9;
    QLineEdit *lineEdit;
    QPushButton *pushButton;
    QGridLayout *gridLayout_2;
    QPushButton *button_right;
    QPushButton *button_forward;
    QPushButton *button_left;
    QPushButton *button_back;
    QPushButton *button_stop;
    QVBoxLayout *verticalLayout_2;
    QPushButton *button_start_mapping;
    QPushButton *button_stop_mapping;
    QPushButton *button_map_reset;
    QLineEdit *input_file_name;
    QPushButton *button_map_save;
    QPushButton *button_map_load;
    QSpacerItem *verticalSpacer;
    QGridLayout *gridLayout_3;
    QLabel *label;
    QLabel *label_3;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QLineEdit *lineEdit_3;
    QLineEdit *lineEdit_4;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(808, 629);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        frame = new QFrame(centralWidget);
        frame->setObjectName(QStringLiteral("frame"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setMinimumSize(QSize(600, 450));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(frame, 2, 0, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        pushButton_9 = new QPushButton(centralWidget);
        pushButton_9->setObjectName(QStringLiteral("pushButton_9"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pushButton_9->sizePolicy().hasHeightForWidth());
        pushButton_9->setSizePolicy(sizePolicy1);
        pushButton_9->setMinimumSize(QSize(162, 0));

        verticalLayout->addWidget(pushButton_9);

        lineEdit = new QLineEdit(centralWidget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        sizePolicy1.setHeightForWidth(lineEdit->sizePolicy().hasHeightForWidth());
        lineEdit->setSizePolicy(sizePolicy1);
        lineEdit->setMinimumSize(QSize(162, 0));

        verticalLayout->addWidget(lineEdit);

        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        sizePolicy1.setHeightForWidth(pushButton->sizePolicy().hasHeightForWidth());
        pushButton->setSizePolicy(sizePolicy1);
        pushButton->setMinimumSize(QSize(162, 0));

        verticalLayout->addWidget(pushButton);


        gridLayout->addLayout(verticalLayout, 1, 1, 1, 1);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setSpacing(6);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        button_right = new QPushButton(centralWidget);
        button_right->setObjectName(QStringLiteral("button_right"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(button_right->sizePolicy().hasHeightForWidth());
        button_right->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(button_right, 1, 2, 1, 1);

        button_forward = new QPushButton(centralWidget);
        button_forward->setObjectName(QStringLiteral("button_forward"));
        sizePolicy2.setHeightForWidth(button_forward->sizePolicy().hasHeightForWidth());
        button_forward->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(button_forward, 0, 1, 1, 1);

        button_left = new QPushButton(centralWidget);
        button_left->setObjectName(QStringLiteral("button_left"));
        sizePolicy2.setHeightForWidth(button_left->sizePolicy().hasHeightForWidth());
        button_left->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(button_left, 1, 0, 1, 1);

        button_back = new QPushButton(centralWidget);
        button_back->setObjectName(QStringLiteral("button_back"));
        sizePolicy2.setHeightForWidth(button_back->sizePolicy().hasHeightForWidth());
        button_back->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(button_back, 2, 1, 1, 1);

        button_stop = new QPushButton(centralWidget);
        button_stop->setObjectName(QStringLiteral("button_stop"));
        sizePolicy2.setHeightForWidth(button_stop->sizePolicy().hasHeightForWidth());
        button_stop->setSizePolicy(sizePolicy2);

        gridLayout_2->addWidget(button_stop, 1, 1, 1, 1);


        gridLayout->addLayout(gridLayout_2, 1, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        button_start_mapping = new QPushButton(centralWidget);
        button_start_mapping->setObjectName(QStringLiteral("button_start_mapping"));
        sizePolicy1.setHeightForWidth(button_start_mapping->sizePolicy().hasHeightForWidth());
        button_start_mapping->setSizePolicy(sizePolicy1);
        button_start_mapping->setMinimumSize(QSize(162, 0));

        verticalLayout_2->addWidget(button_start_mapping);

        button_stop_mapping = new QPushButton(centralWidget);
        button_stop_mapping->setObjectName(QStringLiteral("button_stop_mapping"));
        sizePolicy1.setHeightForWidth(button_stop_mapping->sizePolicy().hasHeightForWidth());
        button_stop_mapping->setSizePolicy(sizePolicy1);
        button_stop_mapping->setMinimumSize(QSize(162, 0));

        verticalLayout_2->addWidget(button_stop_mapping);

        button_map_reset = new QPushButton(centralWidget);
        button_map_reset->setObjectName(QStringLiteral("button_map_reset"));
        sizePolicy1.setHeightForWidth(button_map_reset->sizePolicy().hasHeightForWidth());
        button_map_reset->setSizePolicy(sizePolicy1);
        button_map_reset->setMinimumSize(QSize(162, 0));

        verticalLayout_2->addWidget(button_map_reset);

        input_file_name = new QLineEdit(centralWidget);
        input_file_name->setObjectName(QStringLiteral("input_file_name"));
        sizePolicy1.setHeightForWidth(input_file_name->sizePolicy().hasHeightForWidth());
        input_file_name->setSizePolicy(sizePolicy1);
        input_file_name->setMinimumSize(QSize(162, 0));

        verticalLayout_2->addWidget(input_file_name);

        button_map_save = new QPushButton(centralWidget);
        button_map_save->setObjectName(QStringLiteral("button_map_save"));
        sizePolicy1.setHeightForWidth(button_map_save->sizePolicy().hasHeightForWidth());
        button_map_save->setSizePolicy(sizePolicy1);
        button_map_save->setMinimumSize(QSize(162, 0));

        verticalLayout_2->addWidget(button_map_save);

        button_map_load = new QPushButton(centralWidget);
        button_map_load->setObjectName(QStringLiteral("button_map_load"));

        verticalLayout_2->addWidget(button_map_load);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout_3->addWidget(label, 0, 0, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout_3->addWidget(label_3, 2, 0, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout_3->addWidget(label_2, 1, 0, 1, 1);

        lineEdit_2 = new QLineEdit(centralWidget);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));
        sizePolicy1.setHeightForWidth(lineEdit_2->sizePolicy().hasHeightForWidth());
        lineEdit_2->setSizePolicy(sizePolicy1);
        lineEdit_2->setFocusPolicy(Qt::NoFocus);
        lineEdit_2->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_2, 0, 1, 1, 1);

        lineEdit_3 = new QLineEdit(centralWidget);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));
        sizePolicy1.setHeightForWidth(lineEdit_3->sizePolicy().hasHeightForWidth());
        lineEdit_3->setSizePolicy(sizePolicy1);
        lineEdit_3->setFocusPolicy(Qt::NoFocus);
        lineEdit_3->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_3, 1, 1, 1, 1);

        lineEdit_4 = new QLineEdit(centralWidget);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));
        sizePolicy1.setHeightForWidth(lineEdit_4->sizePolicy().hasHeightForWidth());
        lineEdit_4->setSizePolicy(sizePolicy1);
        lineEdit_4->setFocusPolicy(Qt::NoFocus);
        lineEdit_4->setReadOnly(true);

        gridLayout_3->addWidget(lineEdit_4, 2, 1, 1, 1);


        verticalLayout_2->addLayout(gridLayout_3);


        gridLayout->addLayout(verticalLayout_2, 2, 1, 1, 1);

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
        pushButton_9->setText(QApplication::translate("MainWindow", "Start", Q_NULLPTR));
        pushButton->setText(QApplication::translate("MainWindow", "Set Ip", Q_NULLPTR));
        button_right->setText(QApplication::translate("MainWindow", "Right", Q_NULLPTR));
        button_forward->setText(QApplication::translate("MainWindow", "Forward", Q_NULLPTR));
        button_left->setText(QApplication::translate("MainWindow", "Left", Q_NULLPTR));
        button_back->setText(QApplication::translate("MainWindow", "Back", Q_NULLPTR));
        button_stop->setText(QApplication::translate("MainWindow", "Stop", Q_NULLPTR));
        button_start_mapping->setText(QApplication::translate("MainWindow", "Start mapping", Q_NULLPTR));
        button_stop_mapping->setText(QApplication::translate("MainWindow", "Stop mapping", Q_NULLPTR));
        button_map_reset->setText(QApplication::translate("MainWindow", "Clear map", Q_NULLPTR));
        input_file_name->setInputMask(QString());
        input_file_name->setText(QApplication::translate("MainWindow", "filename.yaml", Q_NULLPTR));
        button_map_save->setText(QApplication::translate("MainWindow", "Save map", Q_NULLPTR));
        button_map_load->setText(QApplication::translate("MainWindow", "Load map", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "X:", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Rot:", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "Y:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
