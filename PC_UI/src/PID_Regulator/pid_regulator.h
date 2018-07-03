#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include <QWidget>
#include "pid_regulator.h"
#include "ui_pid_regulator.h"
class SerialPortSetting;
class QSerialPort;

namespace Ui {
class PID_Regulator;
}

class PID_Regulator : public QWidget
{
    Q_OBJECT

public:
    explicit PID_Regulator(QWidget *parent = 0);
    ~PID_Regulator();
    void Delay_MSec(unsigned int msec);
//    void Delay_MSec_Suspend(unsigned int msec);

private:
    Ui::PID_Regulator *ui;
    bool portIsOpen = false;
    QSerialPort* mySerialPort;
public slots:
    void setBaudRate();
    void setParity();
    void setDataBits();
    void setStopBits();
    void setFlowCtrl();
    void checkAvailablePorts();

    void serialSwitch();
    void queryPIDParams();
    void setPIDParams();
    void readMyCom();

    void setKpValue();
    void setKiValue();
    void setKdValue();
    void setKpSliderValue();
    void setKiSliderValue();
    void setKdSliderValue();
};

#endif // PID_REGULATOR_H
