#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


BUILDDIR = $$PWD/build

OBJECTS_DIR = $${BUILDDIR}
MOC_DIR = $${BUILDDIR}
RCC_DIR = $${BUILDDIR}
UI_DIR = $${BUILDDIR}

TARGET = demoRMR
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    rplidar.cpp \
    CKobuki.cpp

HEADERS  += mainwindow.h \
    rplidar.h \
    CKobuki.h

FORMS    += mainwindow.ui
