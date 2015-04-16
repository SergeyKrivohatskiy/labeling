#-------------------------------------------------
#
# Project created by QtCreator 2015-03-15T18:17:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = labeling
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    base_screen_obstacle.cpp \
    test_point_feature.cpp \
    geom2_to_qt.cpp \
    sim_annealing_opt.cpp

HEADERS  += mainwindow.h \
    geometry.h \
    screen_point_feature.h \
    screen_obstacle.h \
    positions_optimizer.h \
    base_screen_obstacle.h \
    test_point_feature.h \
    geom2_to_qt.h \
    sim_annealing_opt.h \
    point.h \
    segment.h \
    rectangle.h \
    size.h

FORMS    += mainwindow.ui
