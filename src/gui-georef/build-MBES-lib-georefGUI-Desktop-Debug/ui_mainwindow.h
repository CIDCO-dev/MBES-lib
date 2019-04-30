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
    QLabel *labelLeverArm;
    QLabel *labelLeverArmX;
    QLabel *labelLeverArmY;
    QLabel *labelLeverArmZ;
    QLineEdit *lineEditLeverArmX;
    QLineEdit *lineEditLeverArmY;
    QLineEdit *lineEditLeverArmZ;
    QPushButton *LeverArmLoad;
    QPushButton *LeverArmSave;
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
        BrowseInput->setGeometry(QRect(700, 220, 80, 23));
        Process = new QPushButton(centralWidget);
        Process->setObjectName(QString::fromUtf8("Process"));
        Process->setGeometry(QRect(700, 440, 80, 23));
        labelInputFile = new QLabel(centralWidget);
        labelInputFile->setObjectName(QString::fromUtf8("labelInputFile"));
        labelInputFile->setGeometry(QRect(10, 230, 61, 16));
        lineEditInputFile = new QLineEdit(centralWidget);
        lineEditInputFile->setObjectName(QString::fromUtf8("lineEditInputFile"));
        lineEditInputFile->setGeometry(QRect(90, 220, 601, 24));
        labelOutputFile = new QLabel(centralWidget);
        labelOutputFile->setObjectName(QString::fromUtf8("labelOutputFile"));
        labelOutputFile->setGeometry(QRect(10, 300, 71, 16));
        lineEditOutputFile = new QLineEdit(centralWidget);
        lineEditOutputFile->setObjectName(QString::fromUtf8("lineEditOutputFile"));
        lineEditOutputFile->setGeometry(QRect(90, 290, 601, 24));
        BrowseOutput = new QPushButton(centralWidget);
        BrowseOutput->setObjectName(QString::fromUtf8("BrowseOutput"));
        BrowseOutput->setGeometry(QRect(700, 290, 80, 23));
        labelLeverArm = new QLabel(centralWidget);
        labelLeverArm->setObjectName(QString::fromUtf8("labelLeverArm"));
        labelLeverArm->setGeometry(QRect(150, 10, 71, 16));
        labelLeverArmX = new QLabel(centralWidget);
        labelLeverArmX->setObjectName(QString::fromUtf8("labelLeverArmX"));
        labelLeverArmX->setGeometry(QRect(80, 40, 21, 16));
        labelLeverArmY = new QLabel(centralWidget);
        labelLeverArmY->setObjectName(QString::fromUtf8("labelLeverArmY"));
        labelLeverArmY->setGeometry(QRect(80, 70, 21, 16));
        labelLeverArmZ = new QLabel(centralWidget);
        labelLeverArmZ->setObjectName(QString::fromUtf8("labelLeverArmZ"));
        labelLeverArmZ->setGeometry(QRect(80, 100, 21, 16));
        lineEditLeverArmX = new QLineEdit(centralWidget);
        lineEditLeverArmX->setObjectName(QString::fromUtf8("lineEditLeverArmX"));
        lineEditLeverArmX->setGeometry(QRect(100, 30, 113, 24));
        lineEditLeverArmY = new QLineEdit(centralWidget);
        lineEditLeverArmY->setObjectName(QString::fromUtf8("lineEditLeverArmY"));
        lineEditLeverArmY->setGeometry(QRect(100, 60, 113, 24));
        lineEditLeverArmZ = new QLineEdit(centralWidget);
        lineEditLeverArmZ->setObjectName(QString::fromUtf8("lineEditLeverArmZ"));
        lineEditLeverArmZ->setGeometry(QRect(100, 90, 113, 24));
        LeverArmLoad = new QPushButton(centralWidget);
        LeverArmLoad->setObjectName(QString::fromUtf8("LeverArmLoad"));
        LeverArmLoad->setGeometry(QRect(230, 40, 80, 23));
        LeverArmSave = new QPushButton(centralWidget);
        LeverArmSave->setObjectName(QString::fromUtf8("LeverArmSave"));
        LeverArmSave->setGeometry(QRect(230, 70, 80, 23));
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
        labelLeverArm->setText(QApplication::translate("MainWindow", "Lever Arm", 0, QApplication::UnicodeUTF8));
        labelLeverArmX->setText(QApplication::translate("MainWindow", "X", 0, QApplication::UnicodeUTF8));
        labelLeverArmY->setText(QApplication::translate("MainWindow", "Y", 0, QApplication::UnicodeUTF8));
        labelLeverArmZ->setText(QApplication::translate("MainWindow", "Z", 0, QApplication::UnicodeUTF8));
        LeverArmLoad->setText(QApplication::translate("MainWindow", "Load", 0, QApplication::UnicodeUTF8));
        LeverArmSave->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
