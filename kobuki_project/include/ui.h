//
// Created by jakub on 13.2.2019.
//

#ifndef KOBUKI_PROJECT_UI_H
#define KOBUKI_PROJECT_UI_H

#include <QtWidgets>
#include <QMainWindow>
#include <QPushButton>
#include <QFileDialog>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};

#endif //KOBUKI_PROJECT_UI_H
