#include "pid_regulator.h"
#include "ui_pid_regulator.h"

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QDate>
#include <QMessageBox>

#include "crc.h"

PID_Regulator::PID_Regulator(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PID_Regulator)
{
    ui->setupUi(this);
    emit checkAvailablePorts();
    emit setKpValue();
    emit setKiValue();
    emit setKdValue();
}

PID_Regulator::~PID_Regulator()
{
    delete ui;
}
//非阻塞延时
void PID_Regulator::Delay_MSec(unsigned int msec)
{
    QTime _timer = QTime::currentTime().addMSecs(msec);
    while(QTime::currentTime() < _timer)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
//阻塞延时
//void PID_Regulator::Delay_MSec_Suspend(unsigned int msec)
//{
//    QTime _timer = QTime::currentTime();
//    QTime _NowTimer;
//    do{
//        _NowTimer = QTime::currentTime();
//    }while(_timer.msecsTo(_NowTimer) <= msec);
//}

void PID_Regulator::setBaudRate()
{
    qDebug() << "comboBox_baudRate->currentText():" << ui->comboBox_baudRate->currentText() << endl;
    if(portIsOpen)
    {

        if(ui->comboBox_baudRate->currentText() == "115200")
            mySerialPort->setBaudRate(QSerialPort::Baud115200);
        else if(ui->comboBox_baudRate->currentText() == "57600")
            mySerialPort->setBaudRate(QSerialPort::Baud57600);
        else if(ui->comboBox_baudRate->currentText() == "38400")
            mySerialPort->setBaudRate(QSerialPort::Baud38400);
        else if(ui->comboBox_baudRate->currentText() == "19200")
            mySerialPort->setBaudRate(QSerialPort::Baud19200);
        else if(ui->comboBox_baudRate->currentText() == "9600")
            mySerialPort->setBaudRate(QSerialPort::Baud9600);
        else if(ui->comboBox_baudRate->currentText() == "4800")
            mySerialPort->setBaudRate(QSerialPort::Baud4800);
        else if(ui->comboBox_baudRate->currentText() == "2400")
            mySerialPort->setBaudRate(QSerialPort::Baud2400);
        else if(ui->comboBox_baudRate->currentText() == "1200")
            mySerialPort->setBaudRate(QSerialPort::Baud1200);
    }
}

void PID_Regulator::setParity()
{
    qDebug() << "ui->comboBox_parity->currentText():" << ui->comboBox_parity->currentText() << endl;
    if (portIsOpen)
    {
        if (ui->comboBox_parity->currentText() == QString::fromLocal8Bit("无校验"))
            mySerialPort->setParity(QSerialPort::NoParity);
        else if (ui->comboBox_parity->currentText() == QString::fromLocal8Bit("奇校验"))
            mySerialPort->setParity(QSerialPort::OddParity);
        else if (ui->comboBox_parity->currentText() == QString::fromLocal8Bit("偶校验"))
            mySerialPort->setParity(QSerialPort::EvenParity);
    }
}

void PID_Regulator::setDataBits()
{
    if (portIsOpen)
    {
        if (ui->comboBox_wordLength->currentText() == "8")
            mySerialPort->setDataBits(QSerialPort::Data8);
        else if (ui->comboBox_wordLength->currentText() == "7")
            mySerialPort->setDataBits(QSerialPort::Data7);
        else if (ui->comboBox_wordLength->currentText() == "6")
            mySerialPort->setDataBits(QSerialPort::Data6);
        else if (ui->comboBox_wordLength->currentText() == "5")
            mySerialPort->setDataBits(QSerialPort::Data5);
    }
}

void PID_Regulator::setStopBits()
{
    if (portIsOpen)
    {
        if (ui->comboBox_stopBits->currentText() == "1")
            mySerialPort->setStopBits(QSerialPort::OneStop);
        else if (ui->comboBox_stopBits->currentText() == "1.5")
            mySerialPort->setStopBits(QSerialPort::OneAndHalfStop);
        else if (ui->comboBox_stopBits->currentText() == "2")
            mySerialPort->setStopBits(QSerialPort::TwoStop);
    }
}

void PID_Regulator::setFlowCtrl()
{
    if (portIsOpen)
    {
        if (ui->comboBox_flowCtl->currentText() == "off")
            mySerialPort->setFlowControl(QSerialPort::NoFlowControl);
        else if (ui->comboBox_flowCtl->currentText() == "hardware")
            mySerialPort->setFlowControl(QSerialPort::HardwareControl);
        else if (ui->comboBox_flowCtl->currentText() == "xonxoff")
            mySerialPort->setFlowControl(QSerialPort::SoftwareControl);
    }
}

void PID_Regulator::checkAvailablePorts()
{
    foreach (const QSerialPortInfo &Info, QSerialPortInfo::availablePorts())
    {
//        qDebug() << Info.portName() << endl;
        QSerialPort availablePort;
        availablePort.setPortName(Info.portName());
        qDebug() << "availablePort.portName():" <<availablePort.portName() << endl;

        if (availablePort.open(QIODevice::ReadWrite))
        {
            qDebug() << "currentTime:" << QTime::currentTime();
            ui->comboBox_portName->setItemText(0, Info.portName());
            ui->comboBox_portName->setCurrentIndex(0);
            availablePort.close();
        }
    }
}

void PID_Regulator::serialSwitch()
{
    emit checkAvailablePorts();
    static bool checked = true;
    qDebug() << checked << endl;
    if (checked == true)
    {

        checked = false;
        mySerialPort = new QSerialPort(this);
        QString tempStr = ui->comboBox_portName->currentText();
        mySerialPort->setPortName(tempStr);
        if (mySerialPort->open(QIODevice::ReadWrite))
        {
            portIsOpen = true;
            emit setBaudRate();
            emit setParity();
            emit setDataBits();
            emit setStopBits();
            emit setFlowCtrl();
            ui->pushButton_serialSwitch->setText(QString::fromLocal8Bit("Close"));
            ui->textBrowser_log->append(QTime::currentTime().toString()+" Open the serial port successfully!");
            connect(mySerialPort, SIGNAL(readyRead()), this, SLOT(readMyCom()));
        }
        else
        {
            ui->pushButton_serialSwitch->setChecked(false);
            QMessageBox::warning(this,\
                                 QString::fromLocal8Bit("Warning"),\
                                 QString::fromLocal8Bit("No serial USB detected!"),
                                 QMessageBox::Cancel);
        }
    }
    else
    {
        checked = true;
        portIsOpen = false;
        if (mySerialPort->isOpen())
        {
            mySerialPort->close();
            ui->textBrowser_log->append(QTime::currentTime().toString()+" Close the serial port successfully!");
        }
        ui->pushButton_serialSwitch->setText(QString::fromLocal8Bit("Open"));
    }
}

typedef struct{
    unsigned char motor_ID;
    unsigned char PID_Mode;//增量式：1；位置式：0
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Kp_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Ki_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }Kd_value;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }P_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }I_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }D_out_max;
    union{
        unsigned char tempChar[4];
        float tempFloat;
    }PID_out_max;
}PID_struct;

typedef struct{
    unsigned char SOF;
    unsigned char Data_Length;
    unsigned char seq;
    unsigned char crc8;
    unsigned char cmd_ID;
    PID_struct PID;
    union{
        unsigned char tempChar[2];
        int tempInt;
    }crc16;
}MsgsFrame_struct;
MsgsFrame_struct query;
MsgsFrame_struct set;
QByteArray serialReceiveMsgs;
void PID_Regulator::readMyCom()
{
    QByteArray temp = mySerialPort->readAll();
    if(!temp.isEmpty())
    {
        serialReceiveMsgs.append(temp);
        if(serialReceiveMsgs.length() == 37)
        {
            unsigned char * receiveData = (unsigned char *)serialReceiveMsgs.data();
            qDebug() << "serialReceiveMsgs.length()" << serialReceiveMsgs.length() << endl;
            serialReceiveMsgs.clear();
            if(Verify_CRC8_Check_Sum(receiveData,4))
            {
                qDebug() << "CRC8_Check_OK" << endl;
                if(Verify_CRC16_Check_Sum(receiveData,serialReceiveMsgs.length()))
                {
                    qDebug() << "CRC16_Check_OK" << endl;
                }
                query.SOF = *receiveData++;
                query.Data_Length = *receiveData++;
                query.seq = *receiveData++;
                query.crc8 = *receiveData++;
                query.cmd_ID = *receiveData++;
                qDebug() << "SOF_query" << query.SOF << endl;
                qDebug() << "Data_Length_query" << query.Data_Length << endl;

                query.PID.motor_ID = *receiveData++;
                query.PID.PID_Mode = *receiveData++;
                qDebug() << "motor_ID_query" << query.PID.motor_ID << endl;
                qDebug() << "PID_Mode_query" << query.PID.PID_Mode << endl;
                int i=0;
                for(i=0;i<4;i++) query.PID.Kp_value.tempChar[i] = *receiveData++;
                for(i=0;i<4;i++) query.PID.Ki_value.tempChar[i] = *receiveData++;
                for(i=0;i<4;i++) query.PID.Kd_value.tempChar[i] = *receiveData++;
                for(i=0;i<4;i++) query.PID.P_out_max.tempChar[i] = *receiveData++;
                for(i=0;i<4;i++) query.PID.I_out_max.tempChar[i] = *receiveData++;
                for(i=0;i<4;i++) query.PID.D_out_max.tempChar[i] = *receiveData++;
                for(i=0;i<4;i++) query.PID.PID_out_max.tempChar[i] = *receiveData++;

                for(i=0;i<2;i++) query.crc16.tempChar[i] = *receiveData++;

                ui->comboBox_motor_ID->setCurrentIndex(query.PID.motor_ID);
                ui->comboBox_PID_Mode->setCurrentIndex(query.PID.PID_Mode);
                ui->lineEdit_KpValue->setText(QString("%1").arg(query.PID.Kp_value.tempFloat));
                ui->lineEdit_KiValue->setText(QString("%1").arg(query.PID.Ki_value.tempFloat));
                ui->lineEdit_KdValue->setText(QString("%1").arg(query.PID.Kd_value.tempFloat));
                ui->lineEdit_Pout_MAX->setText(QString("%1").arg(query.PID.P_out_max.tempFloat));
                ui->lineEdit_Iout_MAX->setText(QString("%1").arg(query.PID.I_out_max.tempFloat));
                ui->lineEdit_Dout_MAX->setText(QString("%1").arg(query.PID.D_out_max.tempFloat));
                ui->lineEdit_PIDout_MAX->setText(QString("%1").arg(query.PID.PID_out_max.tempFloat));

                ui->textBrowser_log->append(QTime::currentTime().toString() + "PID参数如下：");
                ui->textBrowser_log->append("motor_ID: "+QString("%1").arg(query.PID.motor_ID));
                query.PID.PID_Mode == 1 ? ui->textBrowser_log->append("PID_Mode:增量式")\
                                        :ui->textBrowser_log->append("PID_Mode:位置式");
                ui->textBrowser_log->append("Kp_value = "+QString("%1").arg(query.PID.Kp_value.tempFloat));
                ui->textBrowser_log->append("Ki_value = "+QString("%1").arg(query.PID.Ki_value.tempFloat));
                ui->textBrowser_log->append("Kd_value = "+QString("%1").arg(query.PID.Kd_value.tempFloat));
                ui->textBrowser_log->append("P_out_max = "+QString("%1").arg(query.PID.P_out_max.tempFloat));
                ui->textBrowser_log->append("I_out_max = "+QString("%1").arg(query.PID.I_out_max.tempFloat));
                ui->textBrowser_log->append("D_out_max = "+QString("%1").arg(query.PID.D_out_max.tempFloat));
                ui->textBrowser_log->append("PID_out_max = "+QString("%1").arg(query.PID.PID_out_max.tempFloat));
            }

        }
    }

}

void PID_Regulator::setKpValue()
{
    QString tempStr;
    float tempNum1, tempNum2;
    tempStr = ui->lineEdit_KpScale->text();
    tempNum1 = tempStr.toULong();
    tempNum2 = ui->horizontalSlider_Kp->value();
    tempNum1 *= tempNum2;
    tempStr = QString("%1").arg(tempNum1/10000);
    ui->lineEdit_KpValue->setText(tempStr);
}

void PID_Regulator::setKiValue()
{
    QString tempStr;
    float tempNum1, tempNum2;
    tempStr = ui->lineEdit_KiScale->text();
    tempNum1 = tempStr.toULong();
    tempNum2 = ui->horizontalSlider_Ki->value();
    tempNum1 *= tempNum2;
    tempStr = QString("%1").arg(tempNum1/10000);
    ui->lineEdit_KiValue->setText(tempStr);
}

void PID_Regulator::setKdValue()
{
    QString tempStr;
    float tempNum1, tempNum2;
    tempStr = ui->lineEdit_KdScale->text();
    tempNum1 = tempStr.toULong();
    tempNum2 = ui->horizontalSlider_Kd->value();
    tempNum1 *= tempNum2;
    tempStr = QString("%1").arg(tempNum1/10000);
    ui->lineEdit_KdValue->setText(tempStr);
}

void PID_Regulator::setKpSliderValue()
{
    QString tempStr;
    float tempNum1, tempNum2;
    tempStr = ui->lineEdit_KpScale->text();
    tempNum1 = tempStr.toULong();
    tempStr = ui->lineEdit_KpValue->text();
    tempNum2 = tempStr.toFloat();
    tempNum2 = tempNum2*10000/tempNum1;
    ui->horizontalSlider_Kp->setValue((int)tempNum2);
}

void PID_Regulator::setKiSliderValue()
{
    QString tempStr;
    float tempNum1, tempNum2;
    tempStr = ui->lineEdit_KiScale->text();
    tempNum1 = tempStr.toULong();
    tempStr = ui->lineEdit_KiValue->text();
    tempNum2 = tempStr.toFloat();
    tempNum2 = tempNum2*10000/tempNum1;
    ui->horizontalSlider_Ki->setValue((int)tempNum2);
}

void PID_Regulator::setKdSliderValue()
{
    QString tempStr;
    float tempNum1, tempNum2;
    tempStr = ui->lineEdit_KdScale->text();
    tempNum1 = tempStr.toULong();
    tempStr = ui->lineEdit_KdValue->text();
    tempNum2 = tempStr.toFloat();
    tempNum2 = tempNum2*10000/tempNum1;
    ui->horizontalSlider_Kd->setValue((int)tempNum2);
}

union{
    char byte[4];
    float floatValue;
}uF2C;
#define SOF 0xA5
#define DATA_LENGTH_SET_PID_PATAMS 30
#define CMD_ID_SET_PID_PATAMS 0x01

#define DATA_LENGTH_QUERY_PID_PATAMS 1
#define CMD_ID_QUERY_PID_PATAMS 0x00
void PID_Regulator::setPIDParams()
{
    QByteArray sendData;
    //帧开头
    unsigned char j = 0;
    sendData[j++] = SOF;
    sendData[j++] = DATA_LENGTH_SET_PID_PATAMS;
    static unsigned char seq = 0;
    sendData[j++] = seq;
    seq++;
    //crc8
    unsigned char * tempData = (unsigned char *)sendData.data();
    unsigned char crc8 = Get_CRC8_Check_Sum(tempData,j,0xff);
    qDebug() << "crc8" << crc8 << endl;
    for(int i = 0; i<j;i++)
    {
        qDebug() << "crc8 tempData" << i << tempData[i] << endl;
    }
    sendData[j++] = crc8;
    //信息编码ID
    sendData[j++] = CMD_ID_SET_PID_PATAMS;
    //数据
    //Motor_ID
    QString motorID = ui->comboBox_motor_ID->currentText();
    sendData[j++] = motorID.toUShort();
    //PID_Mode
    char pidMode = ui->comboBox_PID_Mode->currentIndex();
    sendData[j++] = pidMode;

    QString tempStr;
    for(int i=0;i<4;i++)
    {
        uF2C.byte[i] = i;
    }
    qDebug() << "iiiii" << uF2C.floatValue   << endl;

    //KpValue
    tempStr = ui->lineEdit_KpValue->text();
    float KpValue = tempStr.toFloat();
    uF2C.floatValue = KpValue;
    for(int i=0;i<4;i++)    sendData[j++] = uF2C.byte[i];
    qDebug() << "lineEdit_KpValue" << uF2C.floatValue   << endl;
    //KiValue
    tempStr = ui->lineEdit_KiValue->text();
    float KiValue = tempStr.toFloat();
    uF2C.floatValue = KiValue;
    for(int i=0;i<4;i++)    sendData[j++] = uF2C.byte[i];
    qDebug() << "lineEdit_KiValue" << uF2C.floatValue   << endl;
    //KdValue
    tempStr = ui->lineEdit_KdValue->text();
    float KdValue = tempStr.toFloat();
    uF2C.floatValue = KdValue;
    for(int i=0;i<4;i++)    sendData[j++] = uF2C.byte[i];
    qDebug() << "lineEdit_KdValue" << uF2C.floatValue   << endl;
    //Pout_MAX
    tempStr = ui->lineEdit_Pout_MAX->text();
    float Pout_MAX = tempStr.toFloat();
    uF2C.floatValue = Pout_MAX;
    for(int i=0;i<4;i++)    sendData[j++] = uF2C.byte[i];
    qDebug() << "Pout_MAX" << uF2C.floatValue   << endl;
    //Iout_MAX
    tempStr = ui->lineEdit_Iout_MAX->text();
    float Iout_MAX = tempStr.toFloat();
    uF2C.floatValue = Iout_MAX;
    for(int i=0;i<4;i++)    sendData[j++] = uF2C.byte[i];
    qDebug() << "Iout_MAX" << uF2C.floatValue   << endl;
    //Dout_MAX
    tempStr = ui->lineEdit_Dout_MAX->text();
    float Dout_MAX = tempStr.toFloat();
    uF2C.floatValue = Dout_MAX;
    for(int i=0;i<4;i++)    sendData[j++] = uF2C.byte[i];
    qDebug() << "Dout_MAX" << uF2C.floatValue   << endl;
    //PID_out_max
    tempStr = ui->lineEdit_PIDout_MAX->text();
    float PIDout_MAX = tempStr.toFloat();
    uF2C.floatValue = PIDout_MAX;
    for(int i=0;i<4;i++)    sendData[j++] = uF2C.byte[i];
    qDebug() << "PIDout_MAX" << uF2C.floatValue   << endl;
    //crc16
    qDebug() << "jj" << j << endl;
    qDebug() << "sendData.length()" << sendData.length() << endl;

    unsigned char * ttempData = (unsigned char *)sendData.data();
    uint16_t crc16 = Get_CRC16_Check_Sum(ttempData,j,0xffff);
    qDebug() << "crc16" << crc16 << endl;
    char crc16_L = crc16&0xff;
    char crc16_H = (crc16>>8)&0xff;
//    qDebug() << "crc16_L" << crc16_L << endl;
//    qDebug() << "crc16_H" << crc16_H << endl;
    sendData[j++] = crc16_L;
    sendData[j] = crc16_H;

    qDebug() << "sendData.length()" << sendData.length() << endl;
    qDebug() << "jj" << j << endl;
    qDebug() << "sendData" << sendData << endl;
    if(portIsOpen == true)
    {
//        ui->textBrowser_log->append("Sending the PID params!");
        ui->textBrowser_log->append(QTime::currentTime().toString()+" 正在设置PID参数...");
        unsigned char i = 3;
        while(i)
        {
            i--;
            mySerialPort->write(sendData);
//            Delay_MSec(2);
        }
//        mySerialPort->write(sendData);
    }
}

void PID_Regulator::queryPIDParams()
{
    QByteArray sendData;
    //帧开头
    unsigned char j = 0;
    sendData[j++] = SOF;
//    qDebug() << "j" << j << endl;
    sendData[j++] = DATA_LENGTH_QUERY_PID_PATAMS;
//    qDebug() << "j" << j << endl;
    static unsigned char seq = 0;
    sendData[j++] = seq;
    seq++;
    //crc8
    unsigned char * tempData = (unsigned char *)sendData.data();
    unsigned char crc8 = Get_CRC8_Check_Sum(tempData,j,0xff);//TODO
    qDebug() << "crc8" << crc8 << endl;
    for(int i = 0; i<j;i++)
    {
        qDebug() << "crc8 tempData" << i << tempData[i] << endl;
    }
    sendData[j++] = crc8;
    //信息编码ID
    sendData[j++] = CMD_ID_QUERY_PID_PATAMS;
    //数据
    //Motor_ID
    QString motorID = ui->comboBox_motor_ID->currentText();
    sendData[j++] = motorID.toUShort();

    //crc16
    unsigned char * ttempData = (unsigned char *)sendData.data();
    uint16_t crc16 = Get_CRC16_Check_Sum(ttempData,j,0xffff);
    qDebug() << "crc16" << crc16 << endl;
    char crc16_H = (crc16>>8)&0x00ff;
    char crc16_L = crc16&0x00ff;
    sendData[j++] = crc16_L;
    sendData[j] = crc16_H;

    qDebug() << "sendData.length()" << sendData.length() << endl;
    qDebug() << "sendData" << sendData << endl;

    if(portIsOpen == true)
    {
//        ui->textBrowser_log->append("Querying the PID params!");
//        ui->textBrowser_log->append(QTime::currentTime().toString());
        ui->textBrowser_log->append(QTime::currentTime().toString()+" 正在查询板载PID参数...");
        unsigned char i = 3;
        while(i)
        {
            i--;
            mySerialPort->write(sendData);
//            Delay_MSec(2);
        }
//        mySerialPort->write(sendData);
    }
}
