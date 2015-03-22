#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <limits>
#include <qelapsedtimer.h>
#include <qpainter.h>
#include "base_screen_obstacle.h"
#include "screen_obstacle.h"
#include "test_point_feature.h"
#include "simple_optimizer.h"

using geom2::point_i;
using geom2::size_i;
using geom2::rectangle_i;
using labeling::screen_obstacle;
using labeling::screen_point_feature;
using labeling::base_screen_obstacle;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    timer(new QTimer()),
    time_to_optimize(100),
    pos_optimizer(new labeling::simple_optimizer())
{
    ui->setupUi(this);

    //TODO remove this block
    {
        geom2::segment_i s;
        s.start.x = s.start.y = 0;
        s.end.x = s.end.y = 500;

        rectangle_i r;
        r.left_bottom.x = 65;
        r.left_bottom.y = 124;
        r.size.h = r.size.w = 123;

        screen_obstacles.push_back(std::unique_ptr<base_screen_obstacle>(new base_screen_obstacle(s)));
        screen_obstacles.push_back(std::unique_ptr<base_screen_obstacle>(new base_screen_obstacle(r)));

        size_i field_size;
        field_size.w = 800;
        field_size.h = 600;
        for(int i = 0; i < 20; ++i)
        {
            point_i pos, speed;
            pos.x = rand() % field_size.w;
            pos.y = rand() % field_size.h;
            speed.x = rand() % 20 - 10;
            speed.y = rand() % 20 - 10;
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

    QElapsedTimer ellapsed_timer;
    ellapsed_timer.start();
    pos_optimizer->best_fit(time_to_optimize);
    qint32 ellapsed_ms = ellapsed_timer.nsecsElapsed() / 1000;

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

    for(auto it = screen_obstacles.begin(); it != screen_obstacles.end(); ++it)
    {
        screen_obstacle *obstacle = (*it).get();
        if(obstacle->get_type() == screen_obstacle::segment)
        {
            const point_i &start = obstacle->get_segment()->start;
            const point_i &end = obstacle->get_segment()->end;
            painter.setPen(QPen(Qt::yellow, 4));
            painter.drawLine(start.x, start.y, end.x, end.y);
        } else {
            const point_i &left_bottom = obstacle->get_box()->left_bottom;
            const size_i &size = obstacle->get_box()->size;
            painter.fillRect(left_bottom.x, left_bottom.y, size.w, size.h, Qt::yellow);
        }
    }

    for(auto it = screen_points.begin(); it != screen_points.end(); ++it)
    {
        screen_point_feature *point = (*it).get();
        painter.setPen(QPen(Qt::blue, 10));
        const point_i &point_pos = point->get_screen_pivot();
        painter.drawPoint(point_pos.x, point_pos.y);
        const point_i &offset = point->get_label_offset();
        const size_i &label_size = point->get_label_size();
        painter.setPen(QPen(Qt::blue, 1));
        painter.drawLine(point_pos.x + offset.x, point_pos.y + offset.y, point_pos.x, point_pos.y);
        painter.setPen(QPen(Qt::blue, 3));
        painter.drawRect(point_pos.x + offset.x, point_pos.y + offset.y, label_size.w, label_size.h);
    }
}

MainWindow::~MainWindow()
{
}
