#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <limits>
#include <qelapsedtimer.h>
#include <qpainter.h>
#include "base_screen_obstacle.h"
#include "screen_obstacle.h"
#include "test_point_feature.h"
#include "sim_annealing_opt.h"
#include "geom2_to_qt.h"

using namespace geom2;
using labeling::screen_obstacle;
using labeling::screen_point_feature;
using labeling::base_screen_obstacle;
using labeling::test_point_feature;
using std::unique_ptr;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    timer(new QTimer()),
    time_to_optimize(5),
    pos_optimizer(new labeling::sim_annealing_opt())
{
    ui->setupUi(this);

    fill_screen(30, 5, 1);

    connect(timer.get(), SIGNAL(timeout()), this, SLOT(update()));
    update();
    timer->start(UPDATE_TIME_MS);
}

void MainWindow::fill_screen(int points_count,
                             int obstacles_count,
                             int max_speed)
{
    size_i field_size{800, 600};
    for(int i = 0; i < points_count; ++i)
    {
        point_i pos(rand() % field_size.w, rand() % field_size.h);
        point_i speed(rand() % (2 * max_speed + 1) - max_speed,
                      rand() % (2 * max_speed + 1) - max_speed);
        auto new_point = new test_point_feature(pos, speed, field_size);
        screen_points.push_back(
                    unique_ptr<screen_point_feature>(new_point));
        pos_optimizer->register_label(new_point);
    }

    for(int i = 0; i < obstacles_count; ++i)
    {
        point_i pos(rand() % field_size.w, rand() % field_size.h);
        size_i size{rand() % 150 + 50, rand() % 20 + 20};
        screen_obstacle *new_obstacle =
                new base_screen_obstacle(rectangle_i{pos, size});
        screen_obstacles.push_back(
                    unique_ptr<screen_obstacle>(new_obstacle));
        pos_optimizer->register_obstacle(new_obstacle);
    }
}

void MainWindow::update()
{
    for(auto &point_u_ptr: screen_points)
    {
        static_cast<test_point_feature&>(*point_u_ptr).update_position();
    }

    QElapsedTimer ellapsed_timer;
    ellapsed_timer.start();
    pos_optimizer->best_fit(time_to_optimize);
    qint32 ellapsed_ms = ellapsed_timer.nsecsElapsed() / 1000 / 1000;

    QString newStatus = QString("time limit: %1 ms\n"
                                "actual time: %2 ms\n"
                                "obstacles count: %3\n"
                                "points count: %4\n")
            .arg(time_to_optimize)
            .arg(ellapsed_ms)
            .arg(screen_obstacles.size())
            .arg(screen_points.size());
    ui->status_label->setText(newStatus);

    QMainWindow::update();
}

void MainWindow::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    for(auto &obstacle_u_ptr: screen_obstacles)
    {
        screen_obstacle *obstacle = obstacle_u_ptr.get();
        if(obstacle->get_type() == screen_obstacle::segment)
        {
            painter.setPen(QPen(Qt::yellow, 4));
            painter.drawLine(to_qt(*obstacle->get_segment()));
        } else {
            painter.fillRect(to_qt(*obstacle->get_box()), Qt::yellow);
        }
    }

    for(auto &point_u_ptr: screen_points)
    {
        screen_point_feature *point = point_u_ptr.get();
        painter.setPen(QPen(Qt::blue, 10));
        painter.drawPoint(to_qt(point->get_screen_pivot()));
        painter.setPen(QPen(Qt::blue, 1));
        QPoint label_left_bottom =
                to_qt(point->get_screen_pivot() +
                      point->get_label_offset());

        painter.drawLine(label_left_bottom,
                         to_qt(point->get_screen_pivot()));
        painter.setPen(QPen(Qt::blue, 3));
        painter.drawRect(QRect(label_left_bottom,
                               to_qt(point->get_label_size())));
    }
}

MainWindow::~MainWindow()
{
}
