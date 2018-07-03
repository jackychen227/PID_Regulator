#include "pid_regulator.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PID_Regulator w;
    w.show();

    return a.exec();
}
