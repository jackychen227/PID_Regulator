#-------------------------------------------------
#
# Project created by QtCreator 2018-05-24T20:53:45
#
#-------------------------------------------------

QT       += core gui
QT       += serialport
RC_ICONS = RM_Logo.ico

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PID_Regulator
TEMPLATE = app

SOURCES += main.cpp\
        pid_regulator.cpp \
    crc.cpp

HEADERS  += pid_regulator.h \
    crc.h

FORMS    += pid_regulator.ui
