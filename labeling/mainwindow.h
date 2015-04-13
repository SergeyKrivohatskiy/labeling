#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qtimer.h>
#include <memory>
#include <vector>
#include "positions_optimizer.h"

const int UPDATE_TIME_MS = 200;

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
private:
    void fill_screen(int points_count,
                     int obstacles_count,
                     int max_speed);
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
    float time_to_optimize;
    screen_obstacles_t screen_obstacles;
    screen_points_t screen_points;

private:
    int labels_intersection();
};

#endif // MAINWINDOW_H
