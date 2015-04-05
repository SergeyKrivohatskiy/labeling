#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <limits>
#include <qelapsedtimer.h>
#include <qpainter.h>
#include "base_screen_obstacle.h"
#include "screen_obstacle.h"
#include "test_point_feature.h"
#include "simple_optimizer.h"
#include "geom2_to_qt.h"

using namespace geom2;
using labeling::screen_obstacle;
using labeling::screen_point_feature;
using labeling::base_screen_obstacle;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    timer(new QTimer()),
    time_to_optimize(50),
    pos_optimizer(new labeling::simple_optimizer())
{
    ui->setupUi(this);

    //TODO remove this block
    {
        segment_i s{{0, 0}, {500, 500}};

        rectangle_i r{{65, 124}, {125, 125}};

        screen_obstacles.push_back(
                    std::unique_ptr<base_screen_obstacle>(new base_screen_obstacle(s)));
        screen_obstacles.push_back(
                    std::unique_ptr<base_screen_obstacle>(new base_screen_obstacle(r)));

        size_i field_size{800, 600};
        for(int i = 0; i < 14; ++i)
        {
            point_i pos(rand() % field_size.w, rand() % field_size.h);
            point_i speed(rand() % 3 - 1, rand() % 3 - 1);
            auto new_point = new labeling::test_point_feature(pos, speed, field_size);
            screen_points.push_back(std::unique_ptr<screen_point_feature>(new_point));
            pos_optimizer->register_label(new_point);
        }
    }

    connect(timer.get(), SIGNAL(timeout()), this, SLOT(update()));
    update();
    timer->start(UPDATE_TIME_MS);
}

void MainWindow::update()
{
    for(auto it = screen_points.begin(); it != screen_points.end(); ++it)
    {
        static_cast<labeling::test_point_feature&>(**it).update_position();
    }

    double metric_before = static_cast<labeling::simple_optimizer*>
            (pos_optimizer.get())->metric();

    QElapsedTimer ellapsed_timer;
    ellapsed_timer.start();
    pos_optimizer->best_fit(time_to_optimize);
    qint32 ellapsed_ms = ellapsed_timer.nsecsElapsed() / 1000 / 1000;

    double metric_after = static_cast<labeling::simple_optimizer*>
            (pos_optimizer.get())->metric();

    QString newStatus = QString("time limit: %1 ms\n"
                                "actual time: %2 ms\n"
                                "obstacles count: %3\n"
                                "points count: %4\n"
                                "metric before: %5\n"
                                "metric after: %6\n"
                                "metric diff: %7\n")
            .arg(time_to_optimize)
            .arg(ellapsed_ms)
            .arg(screen_obstacles.size())
            .arg(screen_points.size())
            .arg(metric_before)
            .arg(metric_after)
            .arg(metric_before - metric_after);
    ui->status_label->setText(newStatus);

    QMainWindow::update();
}

void MainWindow::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    for(auto it = screen_obstacles.begin(); it != screen_obstacles.end(); ++it)
    {
        screen_obstacle *obstacle = (*it).get();
        if(obstacle->get_type() == screen_obstacle::segment)
        {
            painter.setPen(QPen(Qt::yellow, 4));
            painter.drawLine(to_qt(*obstacle->get_segment()));
        } else {
            painter.fillRect(to_qt(*obstacle->get_box()), Qt::yellow);
        }
    }

    for(auto it = screen_points.begin(); it != screen_points.end(); ++it)
    {
        screen_point_feature *point = (*it).get();
        painter.setPen(QPen(Qt::blue, 10));
        painter.drawPoint(to_qt(point->get_screen_pivot()));
        painter.setPen(QPen(Qt::blue, 1));
        QPoint label_left_bottom =
                to_qt(point->get_screen_pivot() + point->get_label_offset());
        painter.drawLine(label_left_bottom,
                         to_qt(point->get_screen_pivot()));
        painter.setPen(QPen(Qt::blue, 3));
        painter.drawRect(QRect(label_left_bottom, to_qt(point->get_label_size())));
    }
}

MainWindow::~MainWindow()
{
}
