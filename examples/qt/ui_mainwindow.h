/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"
#include "../sharedMemory.h"

extern pSHM smem;

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget;
    QWidget *tab;
    QCustomPlot *CP_RESIDUAL1;
    QCustomPlot *CP_RESIDUAL2;
    QCustomPlot *CP_RESIDUAL3;
    QCustomPlot *CP_RESIDUAL4;
    QWidget *tab_2;
    QCustomPlot *CP_CUSTOM2;
    QCustomPlot *CP_CUSTOM3;
    QCustomPlot *CP_CUSTOM4;
    QCustomPlot *CP_CUSTOM1;
    QGroupBox *GB_control;
    QPushButton *BT_START;
    QPushButton *BT_SWING;
    QPushButton *BT_CUSTOM1;
    QPushButton *BT_CUSTOM2;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(2098, 1002);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(470, 10, 1621, 931));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        CP_RESIDUAL1 = new QCustomPlot(tab);
        CP_RESIDUAL1->setObjectName(QStringLiteral("CP_RESIDUAL1"));
        CP_RESIDUAL1->setGeometry(QRect(20, 20, 770, 410));
        CP_RESIDUAL2 = new QCustomPlot(tab);
        CP_RESIDUAL2->setObjectName(QStringLiteral("CP_RESIDUAL2"));
        CP_RESIDUAL2->setGeometry(QRect(820, 20, 770, 410));
        CP_RESIDUAL3 = new QCustomPlot(tab);
        CP_RESIDUAL3->setObjectName(QStringLiteral("CP_RESIDUAL3"));
        CP_RESIDUAL3->setGeometry(QRect(20, 470, 770, 410));
        CP_RESIDUAL4 = new QCustomPlot(tab);
        CP_RESIDUAL4->setObjectName(QStringLiteral("CP_RESIDUAL4"));
        CP_RESIDUAL4->setGeometry(QRect(820, 470, 770, 410));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        CP_CUSTOM2 = new QCustomPlot(tab_2);
        CP_CUSTOM2->setObjectName(QStringLiteral("CP_CUSTOM2"));
        CP_CUSTOM2->setGeometry(QRect(820, 20, 770, 410));
        CP_CUSTOM3 = new QCustomPlot(tab_2);
        CP_CUSTOM3->setObjectName(QStringLiteral("CP_CUSTOM3"));
        CP_CUSTOM3->setGeometry(QRect(20, 470, 770, 410));
        CP_CUSTOM4 = new QCustomPlot(tab_2);
        CP_CUSTOM4->setObjectName(QStringLiteral("CP_CUSTOM4"));
        CP_CUSTOM4->setGeometry(QRect(820, 470, 770, 410));
        CP_CUSTOM1 = new QCustomPlot(tab_2);
        CP_CUSTOM1->setObjectName(QStringLiteral("CP_CUSTOM1"));
        CP_CUSTOM1->setGeometry(QRect(20, 20, 770, 410));
        tabWidget->addTab(tab_2, QString());
        GB_control = new QGroupBox(centralWidget);
        GB_control->setObjectName(QStringLiteral("GB_control"));
        GB_control->setGeometry(QRect(30, 30, 401, 241));
        BT_START = new QPushButton(GB_control);
        BT_START->setObjectName(QStringLiteral("BT_START"));
        BT_START->setGeometry(QRect(10, 40, 181, 61));
        BT_SWING = new QPushButton(GB_control);
        BT_SWING->setObjectName(QStringLiteral("BT_SWING"));
        BT_SWING->setGeometry(QRect(210, 40, 181, 61));
        BT_CUSTOM1 = new QPushButton(GB_control);
        BT_CUSTOM1->setObjectName(QStringLiteral("BT_CUSTOM1"));
        BT_CUSTOM1->setGeometry(QRect(10, 120, 181, 61));
        BT_CUSTOM2 = new QPushButton(GB_control);
        BT_CUSTOM2->setObjectName(QStringLiteral("BT_CUSTOM2"));
        BT_CUSTOM2->setGeometry(QRect(210, 120, 181, 61));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 2098, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Residual", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Custom", Q_NULLPTR));
        GB_control->setTitle(QApplication::translate("MainWindow", "CONTROL", Q_NULLPTR));
        BT_START->setText(QApplication::translate("MainWindow", "START", Q_NULLPTR));
        BT_SWING->setText(QApplication::translate("MainWindow", "SWING", Q_NULLPTR));
        BT_CUSTOM1->setText(QApplication::translate("MainWindow", "CUSTOM1", Q_NULLPTR));
        BT_CUSTOM2->setText(QApplication::translate("MainWindow", "CUSTOM2", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
