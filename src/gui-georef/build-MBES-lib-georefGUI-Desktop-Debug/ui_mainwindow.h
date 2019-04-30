/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *BrowseInput;
    QPushButton *Process;
    QLabel *labelInputFile;
    QLineEdit *lineEditInputFile;
    QLabel *labelOutputFile;
    QLineEdit *lineEditOutputFile;
    QPushButton *BrowseOutput;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(791, 526);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        BrowseInput = new QPushButton(centralWidget);
        BrowseInput->setObjectName(QString::fromUtf8("BrowseInput"));
        BrowseInput->setGeometry(QRect(700, 60, 80, 23));
        Process = new QPushButton(centralWidget);
        Process->setObjectName(QString::fromUtf8("Process"));
        Process->setGeometry(QRect(700, 440, 80, 23));
        labelInputFile = new QLabel(centralWidget);
        labelInputFile->setObjectName(QString::fromUtf8("labelInputFile"));
        labelInputFile->setGeometry(QRect(10, 70, 61, 16));
        lineEditInputFile = new QLineEdit(centralWidget);
        lineEditInputFile->setObjectName(QString::fromUtf8("lineEditInputFile"));
        lineEditInputFile->setGeometry(QRect(90, 60, 601, 24));
        labelOutputFile = new QLabel(centralWidget);
        labelOutputFile->setObjectName(QString::fromUtf8("labelOutputFile"));
        labelOutputFile->setGeometry(QRect(10, 220, 71, 16));
        lineEditOutputFile = new QLineEdit(centralWidget);
        lineEditOutputFile->setObjectName(QString::fromUtf8("lineEditOutputFile"));
        lineEditOutputFile->setGeometry(QRect(90, 210, 601, 24));
        BrowseOutput = new QPushButton(centralWidget);
        BrowseOutput->setObjectName(QString::fromUtf8("BrowseOutput"));
        BrowseOutput->setGeometry(QRect(700, 210, 80, 23));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 791, 20));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        BrowseInput->setText(QApplication::translate("MainWindow", "Browse", 0, QApplication::UnicodeUTF8));
        Process->setText(QApplication::translate("MainWindow", "Process", 0, QApplication::UnicodeUTF8));
        labelInputFile->setText(QApplication::translate("MainWindow", "Input File", 0, QApplication::UnicodeUTF8));
        labelOutputFile->setText(QApplication::translate("MainWindow", "Output File", 0, QApplication::UnicodeUTF8));
        BrowseOutput->setText(QApplication::translate("MainWindow", "Browse", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
