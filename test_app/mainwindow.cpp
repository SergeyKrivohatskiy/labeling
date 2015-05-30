#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <limits>
#include <qelapsedtimer.h>
#include <qpainter.h>
#include <QMouseEvent>
#include "base_screen_obstacle.h"
#include "labeling/screen_obstacle.h"
#include "test_point_feature.h"
#include "labeling/sim_annealing_opt.h"
#include "labeling/ray_intersection_opt.h"
#include "geom2_to_qt.h"
#include "labeling/utils.h"

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
    pos_optimizer(new labeling::ray_intersection_opt())
//    pos_optimizer(new labeling::sim_annealing_opt())
{
    ui->setupUi(this);

    field_size.w = size().width();
    field_size.h = size().height();
    fill_screen(INIT_POINTS_COUNT, INIT_OBSTACLES_COUNT);

    connect(timer.get(), SIGNAL(timeout()), this, SLOT(update()));
    update();
    timer->start(UPDATE_TIME_MS);
}

void MainWindow::add_point(const point_i &pos)
{
    double to_0_1 = 1.0 / RAND_MAX;
    point_d speed(rand() * to_0_1 * (2 * MAX_POINT_SPEED) - MAX_POINT_SPEED,
                  rand() * to_0_1 * (2 * MAX_POINT_SPEED) - MAX_POINT_SPEED);
    auto new_point = new test_point_feature(pos,
                                            speed,
                                            field_size,
                                            rand() * to_0_1 < FIXED_POINT_P,
                                            rand() * to_0_1 * 2 * MAX_POINT_ROT
                                            - MAX_POINT_ROT);
    screen_points.push_back(
                unique_ptr<screen_point_feature>(new_point));
    pos_optimizer->register_label(new_point);
}

void MainWindow::add_obstacle()
{
    point_i pos(rand() % field_size.w, rand() % field_size.h);
    size_i size{rand() % 150 + 50, rand() % 20 + 20};
    screen_obstacle *new_obstacle =
            new base_screen_obstacle(rectangle_i{pos, size});
    screen_obstacles.push_back(
                unique_ptr<screen_obstacle>(new_obstacle));
    pos_optimizer->register_obstacle(new_obstacle);
}

void MainWindow::fill_screen(int points_count,
                             int obstacles_count)
{
    for(int i = 0; i < points_count; ++i)
    {
        point_i pos(rand() % field_size.w, rand() % field_size.h);
        add_point(pos);
    }

    for(int i = 0; i < obstacles_count; ++i)
    {
        add_obstacle();
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
    pos_optimizer->best_fit(TIME_TO_OPTIMIZE);
    qint32 ellapsed_ms = ellapsed_timer.nsecsElapsed() / 1000 / 1000;

    QString newStatus = QString("time limit: %1 ms\n"
                                "actual time: %2 ms\n"
                                "obstacles count: %3\n"
                                "points count: %4\n")
            .arg(TIME_TO_OPTIMIZE)
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
        test_point_feature *point =
                static_cast<test_point_feature*>(point_u_ptr.get());

        auto point_color = point->is_label_fixed() ? Qt::darkCyan : Qt::blue;
        painter.setPen(QPen(point_color, 10));
        painter.drawPoint(to_qt(point->get_screen_pivot()));
        painter.setPen(QPen(point_color, 1));
        QPoint label_left_bottom =
                to_qt(point->get_screen_pivot() +
                      point->get_label_offset());
        point_i label_center = labeling::to_label_rect(point).center();

        point_i intersection_point;
        if(seg_rect_intersection(segment_i{label_center,
                                 point->get_screen_pivot()},
                                 labeling::to_label_rect(point),
                                 &intersection_point,
                                 static_cast<point_i *>(nullptr)))
        {
            painter.drawLine(to_qt(intersection_point),
                             to_qt(point->get_screen_pivot()));
            painter.setPen(QPen(point_color, 4));
            painter.drawPoint(to_qt(intersection_point));
        } // else- line is under the label

        painter.setPen(QPen(point_color, 3));
        painter.drawRect(QRect(label_left_bottom,
                               to_qt(point->get_label_size())));
        painter.setPen(QPen(Qt::red, 1));
        for(const point_i &track_point: point->get_track())
        {
            painter.drawPoint(to_qt(track_point));
        }
    }
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    //    Left button on label make it fixed
    //    Left button on point removes it
    //    Left button on empty space adds new point
    if(event->button() == Qt::LeftButton)
    {
        point_i pos(event->x(), event->y());
        for(auto it = screen_points.begin(); it != screen_points.end(); ++ it)
        {
            test_point_feature* point =
                    dynamic_cast<test_point_feature*>((*it).get());
            if(points_distance(
                        point->
                        get_screen_pivot(),
                        pos) < 10)
            {
                pos_optimizer->unregister_label(point);
                screen_points.erase(it);
                return;
            }
            rectangle_i label_rect = labeling::to_label_rect(point);
            if(point_in_rect(pos, label_rect))
            {
                point->set_fixed(!point->is_label_fixed());
                return;
            }
        }
        add_point(pos);
        return;
    }
    QMainWindow::mousePressEvent(event);
}

MainWindow::~MainWindow()
{
}
