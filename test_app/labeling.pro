#-------------------------------------------------
#
# Project created by QtCreator 2015-03-15T18:17:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = labeling
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    base_screen_obstacle.cpp \
    test_point_feature.cpp \
    geom2_to_qt.cpp \
    labeling/sim_annealing_opt.cpp \
    labeling/utils.cpp \
    labeling/ray_intersection_opt.cpp \
    labeling/base_optimizer.cpp

HEADERS  += mainwindow.h \
    base_screen_obstacle.h \
    test_point_feature.h \
    geom2_to_qt.h \
    labeling/geometry.h \
    labeling/point.h \
    labeling/positions_optimizer.h \
    labeling/rectangle.h \
    labeling/screen_obstacle.h \
    labeling/screen_point_feature.h \
    labeling/segment.h \
    labeling/sim_annealing_opt.h \
    labeling/size.h \
    labeling/utils.h \
    labeling/ray_intersection_opt.h \
    labeling/base_optimizer.h

FORMS    += mainwindow.ui
