#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qtimer.h>
#include <memory>
#include <vector>
#include "labeling/positions_optimizer.h"

const int UPDATE_TIME_MS = 100;
const float TIME_TO_OPTIMIZE = 5;
const double MAX_POINT_SPEED = 0.7;
const double MAX_POINT_ROT = 2 * 3.14 / 360 * 0.2;
const double FIXED_POINT_P = 0;
const int INIT_POINTS_COUNT = 50;
const int INIT_OBSTACLES_COUNT = 10;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void paintEvent(QPaintEvent *);
    void mousePressEvent(QMouseEvent *);
private:
    void add_obstacle();
    void add_point(const geom2::point_i &pos);
    void fill_screen(int points_count,
                     int obstacles_count);
private slots:
    void update();

private:
    typedef std::vector<std::unique_ptr<labeling::screen_obstacle>>
        screen_obstacles_t;
    typedef std::vector<std::unique_ptr<labeling::screen_point_feature>>
        screen_points_t;
private:
    std::unique_ptr<Ui::MainWindow> ui;
    std::unique_ptr<QTimer> timer;
    std::unique_ptr<labeling::positions_optimizer> pos_optimizer;
    screen_obstacles_t screen_obstacles;
    screen_points_t screen_points;
    geom2::size_i field_size;
private:
    int labels_intersection();
};

#endif // MAINWINDOW_H
