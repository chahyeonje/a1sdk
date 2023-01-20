#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include <QTimer>
#include <QPen>
#include <iostream>
#include "../sharedMemory.h"

extern pSHM smem;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void GraphInitialize();
    void GraphUdate();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *graphTimer;
    double graphOffset;
};

#endif // MAINWINDOW_H
