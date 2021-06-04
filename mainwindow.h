#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QLabel>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QRegularExpression>

#include <cmath>
#include <qcustomplot.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    const QString PORT_NUMBER = "COM1";
    const QVector<QString> LIST_COMMANDS = {
        "START>", "STOP>", "UTURN>", "RESET>"};
    const QRegularExpression REGEX = QRegularExpression(
                "(\\bOdometrie: \\b)(\\d{1,3})\\s(\\d{1,3})");
    const double L = 0.27/2;


private:
    void setupUart();
    void setupInterface();
    void setupSignals();

    void computePose(double, double);
    void updateFigure();

    QSerialPort *uartPort;
    QByteArray uartReceived;
    QString uartBuffer;

    QVector<double> poseX;
    QVector<double> poseY;
    QVector<double> poseP;

    QCustomPlot *fig;
    QCPGraph *ax;
    QPushButton *buttonStart;
    QPushButton *buttonStop;
    QPushButton *buttonReset;

    QLabel *labelXMin;
    QLabel *labelXMax;
    QLabel *labelYMin;
    QLabel *labelYMax;
    QDoubleSpinBox *spinXMin;
    QDoubleSpinBox *spinXMax;
    QDoubleSpinBox *spinYMin;
    QDoubleSpinBox *spinYMax;

    QHBoxLayout *hboxSpin;
    QHBoxLayout *hbox;
    QVBoxLayout *vbox;

    QWidget *central;

private slots:
    void onClickButtonStart();
    void onClickButtonStop();
    void onClickButtonReset();
    void onChangeSpin();
    void onReceiveUart();
};
#endif // MAINWINDOW_H
