#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GraphInitialize();
    graphTimer = new QTimer();
    connect(graphTimer, SIGNAL(timeout()), this, SLOT(GraphUdate()));
    graphTimer->start(1);
    graphOffset = 1;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::GraphInitialize()
{
    QPen myPen1,myPen2,myPen3,myPen4, dotPen, filterPen;
    myPen1.setWidthF(1);
    filterPen.setStyle(Qt::DotLine);
    filterPen.setWidth(1);
    dotPen.setStyle(Qt::DotLine);
    dotPen.setWidth(20);
    dotPen.setWidthF(2);
    dotPen.setColor(Qt::gray);

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");

    ui->CP_RESIDUAL1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->CP_RESIDUAL2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->CP_RESIDUAL3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->CP_RESIDUAL4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    ui->CP_CUSTOM1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->CP_CUSTOM2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->CP_CUSTOM3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->CP_CUSTOM4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    myPen1.setWidthF(1.5);
    myPen1.setColor(Qt::blue);
    myPen2.setWidthF(1.5);
    myPen2.setColor(Qt::red);
    myPen3.setWidthF(1.5);
    myPen3.setColor(Qt::green);
    myPen4.setWidthF(1.5);
    myPen4.setColor(Qt::yellow);

    ui->CP_RESIDUAL1->legend->setVisible(true);
    ui->CP_RESIDUAL1->legend->setFont(QFont("Helvetica",9));
    ui->CP_RESIDUAL1->addGraph();
    ui->CP_RESIDUAL1->graph(0)->setPen(myPen1);
    ui->CP_RESIDUAL1->graph(0)->setName("Residual-FL");

    ui->CP_RESIDUAL2->legend->setVisible(true);
    ui->CP_RESIDUAL2->legend->setFont(QFont("Helvetica",9));
    ui->CP_RESIDUAL2->addGraph();
    ui->CP_RESIDUAL2->graph(0)->setPen(myPen1);
    ui->CP_RESIDUAL2->graph(0)->setName("Residual-FR");

    ui->CP_RESIDUAL3->legend->setVisible(true);
    ui->CP_RESIDUAL3->legend->setFont(QFont("Helvetica",9));
    ui->CP_RESIDUAL3->addGraph();
    ui->CP_RESIDUAL3->graph(0)->setPen(myPen1);
    ui->CP_RESIDUAL3->graph(0)->setName("Residual-RL");

    ui->CP_RESIDUAL4->legend->setVisible(true);
    ui->CP_RESIDUAL4->legend->setFont(QFont("Helvetica",9));
    ui->CP_RESIDUAL4->addGraph();
    ui->CP_RESIDUAL4->graph(0)->setPen(myPen1);
    ui->CP_RESIDUAL4->graph(0)->setName("Residual-RR");

    ui->CP_CUSTOM1->legend->setVisible(true);
    ui->CP_CUSTOM1->legend->setFont(QFont("Helvetica",9));
    ui->CP_CUSTOM1->addGraph();
    ui->CP_CUSTOM1->graph(0)->setPen(myPen1);
    ui->CP_CUSTOM1->graph(0)->setName("Residual-RR1");

    ui->CP_CUSTOM2->legend->setVisible(true);
    ui->CP_CUSTOM2->legend->setFont(QFont("Helvetica",9));
    ui->CP_CUSTOM2->addGraph();
    ui->CP_CUSTOM2->graph(0)->setPen(myPen1);
    ui->CP_CUSTOM2->graph(0)->setName("Residual-RR2");

    ui->CP_CUSTOM3->legend->setVisible(true);
    ui->CP_CUSTOM3->legend->setFont(QFont("Helvetica",9));
    ui->CP_CUSTOM3->addGraph();
    ui->CP_CUSTOM3->graph(0)->setPen(myPen1);
    ui->CP_CUSTOM3->graph(0)->setName("Residual-RR3");

    ui->CP_CUSTOM4->legend->setVisible(true);
    ui->CP_CUSTOM4->legend->setFont(QFont("Helvetica",9));
    ui->CP_CUSTOM4->addGraph();
    ui->CP_CUSTOM4->graph(0)->setPen(myPen1);
    ui->CP_CUSTOM4->graph(0)->setName("Residual-RR");

    ui->CP_RESIDUAL1->xAxis->setTicker(timeTicker);
    ui->CP_RESIDUAL2->xAxis->setTicker(timeTicker);
    ui->CP_RESIDUAL3->xAxis->setTicker(timeTicker);
    ui->CP_RESIDUAL4->xAxis->setTicker(timeTicker);

    ui->CP_CUSTOM1->xAxis->setTicker(timeTicker);
    ui->CP_CUSTOM2->xAxis->setTicker(timeTicker);
    ui->CP_CUSTOM3->xAxis->setTicker(timeTicker);
    ui->CP_CUSTOM4->xAxis->setTicker(timeTicker);

    ui->CP_RESIDUAL1->yAxis->setRange(0,7);
    ui->CP_RESIDUAL2->yAxis->setRange(0,7);
    ui->CP_RESIDUAL3->yAxis->setRange(0,7);
    ui->CP_RESIDUAL4->yAxis->setRange(0,7);

    ui->CP_CUSTOM1->yAxis->setRange(0,7);
    ui->CP_CUSTOM2->yAxis->setRange(0,7);
    ui->CP_CUSTOM3->yAxis->setRange(0,7);
    ui->CP_CUSTOM4->yAxis->setRange(0,7);
}

void MainWindow::GraphUdate()
{
    ui->CP_RESIDUAL1->xAxis->setRange(simTime - graphOffset, simTime);
    ui->CP_RESIDUAL1->graph(0)->addData(simTime, resFL);
    ui->CP_RESIDUAL1->replot();

    ui->CP_RESIDUAL2->xAxis->setRange(simTime - graphOffset, simTime);
    ui->CP_RESIDUAL2->graph(0)->addData(simTime, resFR);
    ui->CP_RESIDUAL2->replot();

    ui->CP_RESIDUAL3->xAxis->setRange(simTime - graphOffset, simTime);
    ui->CP_RESIDUAL3->graph(0)->addData(simTime, resRL);
    ui->CP_RESIDUAL3->replot();

    ui->CP_RESIDUAL4->xAxis->setRange(simTime - graphOffset, simTime);
    ui->CP_RESIDUAL4->graph(0)->addData(simTime, resRR);
    ui->CP_RESIDUAL4->replot();

    ui->CP_CUSTOM1->xAxis->setRange(simTime - graphOffset, simTime);
    ui->CP_CUSTOM1->graph(0)->addData(simTime, resRR1);
    ui->CP_CUSTOM1->replot();

    ui->CP_CUSTOM2->xAxis->setRange(simTime - graphOffset, simTime);
    ui->CP_CUSTOM2->graph(0)->addData(simTime, resRR2);
    ui->CP_CUSTOM2->replot();

    ui->CP_CUSTOM3->xAxis->setRange(simTime - graphOffset, simTime);
    ui->CP_CUSTOM3->graph(0)->addData(simTime, resRR3);
    ui->CP_CUSTOM3->replot();
}

void MainWindow::on_pushButton_clicked()
{
//    std::cout << "START button is clikcked" << std::endl;
//    if(smem->start){smem->start = false;}
//    else{smem->start = true;}
}

void MainWindow::on_pushButton_2_clicked()
{

}

void MainWindow::on_pushButton_3_clicked()
{

}

void MainWindow::on_pushButton_4_clicked()
{

}

