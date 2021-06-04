#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // Instanciate
    uartPort = new QSerialPort();
    uartBuffer = "";
    // Initial pose
    poseX.push_back(0);
    poseY.push_back(0);
    poseP.push_back(0);
    // Initialize application
    setupUart();
    setupInterface();
    setupSignals();
}

MainWindow::~MainWindow(){
    // Close the connection
    uartPort->close();
}

void MainWindow::setupUart(){
    // Configure the serial port
    uartPort->setPortName(PORT_NUMBER);
    uartPort->open(QSerialPort::ReadWrite);
    uartPort->setBaudRate(QSerialPort::Baud9600);
    uartPort->setDataBits(QSerialPort::Data8);
    uartPort->setParity(QSerialPort::NoParity);
    uartPort->setStopBits(QSerialPort::OneStop);
    uartPort->setFlowControl(QSerialPort::NoFlowControl);
}

void MainWindow::setupInterface(){
    /* **************
     * *** Figure ***
     * ************** */
    // Instanciate
    fig = new QCustomPlot;
    ax = fig->addGraph();
    // Seperate points
    ax->setLineStyle(QCPGraph::lsNone);
    // Filled circle, red, 10 px
    ax->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, Qt::red, 10));
    // Sketch initial pose
    ax->setData(poseX, poseY);
    // Set axes limits
    fig->xAxis->setRange(-1, 1);
    fig->yAxis->setRange(-1, 1);

    /* ***************
     * *** Buttons ***
     * *************** */
    buttonStart = new QPushButton("Start");
    buttonStop = new QPushButton("Stop");
    buttonReset = new QPushButton("Reset");

    /* *******************
     * *** Axes limits ***
     * ******************* */
    labelXMin = new QLabel("xmin");
    spinXMin = new QDoubleSpinBox;
    spinXMin->setMinimum(-99);
    spinXMin->setValue(-1.0);
    spinXMin->setSingleStep(0.1);

    labelXMax = new QLabel("xmax");
    spinXMax = new QDoubleSpinBox;
    spinXMax->setMinimum(-99);
    spinXMax->setValue(1);
    spinXMax->setSingleStep(0.1);

    labelYMin = new QLabel("ymin");
    spinYMin = new QDoubleSpinBox;
    spinYMin->setMinimum(-99);
    spinYMin->setValue(-1.0);
    spinYMin->setSingleStep(0.1);

    labelYMax = new QLabel("ymax");
    spinYMax = new QDoubleSpinBox;
    spinYMax->setMinimum(-99);
    spinYMax->setValue(1);
    spinYMax->setSingleStep(0.1);

    hboxSpin = new QHBoxLayout;
    hboxSpin->addWidget(labelXMin);
    hboxSpin->addWidget(spinXMin);
    hboxSpin->addWidget(labelXMax);
    hboxSpin->addWidget(spinXMax);
    hboxSpin->addWidget(labelYMin);
    hboxSpin->addWidget(spinYMin);
    hboxSpin->addWidget(labelYMax);
    hboxSpin->addWidget(spinYMax);

    hbox = new QHBoxLayout;
    hbox->addWidget(buttonStart);
    hbox->addWidget(buttonStop);
    hbox->addWidget(buttonReset);
    hbox->addStretch();
    hbox->addLayout(hboxSpin);

    /* *******************
     * *** Main layout ***
     * ******************* */
    vbox = new QVBoxLayout;
    vbox->addWidget(fig);
    vbox->addLayout(hbox);
    central = new QWidget;
    central->setLayout(vbox);
    setCentralWidget(central);
    setWindowTitle("Qt Odometry");
}

void MainWindow::setupSignals(){
    connect(buttonStart, SIGNAL(clicked()),
            this, SLOT(onClickButtonStart()));
    connect(buttonStop, SIGNAL(clicked()),
            this, SLOT(onClickButtonStop()));
    connect(buttonReset, SIGNAL(clicked()),
            this, SLOT(onClickButtonReset()));
    connect(spinXMin, SIGNAL(valueChanged(double)),
            this, SLOT(onChangeSpin()));
    connect(spinXMax, SIGNAL(valueChanged(double)),
            this, SLOT(onChangeSpin()));
    connect(spinYMin, SIGNAL(valueChanged(double)),
            this, SLOT(onChangeSpin()));
    connect(spinYMax, SIGNAL(valueChanged(double)),
            this, SLOT(onChangeSpin()));
    connect(uartPort, SIGNAL(readyRead()),
            this, SLOT(onReceiveUart()));
}

void MainWindow::onClickButtonStart(){
    QString command = LIST_COMMANDS[0];
    if(uartPort->isWritable())
        uartPort->write(command.toStdString().c_str());
}

void MainWindow::onClickButtonStop(){
    QString command = LIST_COMMANDS[1];
    if(uartPort->isWritable())
        uartPort->write(command.toStdString().c_str());
}

void MainWindow::onClickButtonReset(){
    QString command = LIST_COMMANDS[3];
    if(uartPort->isWritable())
        uartPort->write(command.toStdString().c_str());

    // Clear poses and figure
    poseX.clear();
    poseY.clear();
    poseP.clear();
    poseX.push_back(0);
    poseY.push_back(0);
    poseP.push_back(0);
    updateFigure();
}

void MainWindow::onChangeSpin(){
    // Change axes limits
    fig->xAxis->setRange(spinXMin->value(), spinXMax->value());
    fig->yAxis->setRange(spinYMin->value(), spinYMax->value());
    fig->replot();
}

void MainWindow::onReceiveUart(){
    // Store received characters in a buffer
    uartReceived = uartPort->readAll();
    uartBuffer += QString::fromStdString(uartReceived.toStdString());
    // Process received data if it is a complete line
    if(uartBuffer.endsWith("\n")){
        // Process data only if it passes the filter
        QRegularExpressionMatch match = REGEX.match(uartBuffer);
        if (match.hasMatch()) {
            computePose(match.captured(2).toInt(),
                      match.captured(3).toInt());
            updateFigure();
        }
        // Clear buffer
        uartBuffer = "";
    }
}

void MainWindow::computePose(double d_left, double d_right){
    // Compute new pose according to the kinematics model
    double v = (d_right + d_left) / 2.0;
    double w = (d_right - d_left) / (2 * L);
    double xn = poseX.back();
    double yn = poseY.back();
    double pn = poseP.back();
    double dx = v * std::cos(pn);
    double dy = v * std::sin(pn);
    double dp = w;

    // Add new pose to the list
    poseX.push_back(xn+dx);
    poseY.push_back(yn+dy);
    poseP.push_back(pn+dp);
}

void MainWindow::updateFigure(){
    ax->setData(poseX, poseY);
    fig->replot();
}
