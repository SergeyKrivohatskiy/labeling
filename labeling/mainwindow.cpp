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
        geom2::segment_i s{{0, 0}, {500, 500}};

        rectangle_i r{{65, 124}, {125, 125}};

        screen_obstacles.push_back(
                    std::unique_ptr<base_screen_obstacle>(new base_screen_obstacle(s)));
        screen_obstacles.push_back(
                    std::unique_ptr<base_screen_obstacle>(new base_screen_obstacle(r)));

        size_i field_size{800, 600};
        for(int i = 0; i < 20; ++i)
        {
            point_i pos(rand() % field_size.w, rand() % field_size.h);
            point_i speed(rand() % 20 - 10, rand() % 20 - 10);
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

    int summ_intersection_before = labels_intersection();

    QElapsedTimer ellapsed_timer;
    ellapsed_timer.start();
    pos_optimizer->best_fit(time_to_optimize);
    qint32 ellapsed_ms = ellapsed_timer.nsecsElapsed() / 1000;

    int summ_intersection_after = labels_intersection();

    QString newStatus = QString("time limit: %1 ms\n"
                                "actual time: %2 ms\n"
                                "obstacles count: %3\n"
                                "points count: %4\n"
                                "labels intersection before: %5\n"
                                "labels intersection after: %6\n"
                                "labels intersection diff: %7\n")
            .arg(time_to_optimize)
            .arg(ellapsed_ms)
            .arg(screen_obstacles.size())
            .arg(screen_points.size())
            .arg(summ_intersection_before)
            .arg(summ_intersection_after)
            .arg(summ_intersection_before - summ_intersection_after);
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
        point_i label_left_bottom = point_pos + offset;
        painter.drawLine(label_left_bottom.x, label_left_bottom.y,
                         point_pos.x, point_pos.y);
        painter.setPen(QPen(Qt::blue, 3));
        painter.drawRect(label_left_bottom.x, label_left_bottom.y,
                         label_size.w, label_size.h);
    }
}

int MainWindow::labels_intersection()
{
    int summ = 0;
    for(auto &point_ptr1: screen_points)
    {
        for(auto &point_ptr2: screen_points)
        {
            if(point_ptr1 == point_ptr2)
            {
                continue;
            }
            summ += labeling::labels_intersection(*point_ptr1, *point_ptr2);
        }
    }
    return summ;
}

MainWindow::~MainWindow()
{
}
