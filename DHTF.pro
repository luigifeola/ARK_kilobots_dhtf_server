#-------------------------------------------------
#
# Project created by QtCreator 2018-01-01T07:35:48
#
#-------------------------------------------------

QT       -= gui

QT += widgets network

TARGET = DHTF
TEMPLATE = lib

DEFINES += DHTFEXP_LIBRARY

SOURCES += \
    kilobot.cpp \
    serverStuff.cpp \
    dhtfEnvironment.cpp \
    dhtfExperiment.cpp

HEADERS +=\
    kilobot.h \
    serverStuff.h \
    kilobotexperiment.h \
    kilobotenvironment.h \
    global.h \
    dhtfEnvironment.h \
    dhtfExperiment.h \
    area.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib \
        -lopencv_core
