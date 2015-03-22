#include "mainwindow.h"
#include <QApplication>
#include "base_screen_obstacle.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
