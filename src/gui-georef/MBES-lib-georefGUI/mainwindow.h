/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <Eigen/Dense>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private slots:

//    void on_Browse_clicked();

    void on_Process_clicked();

    void on_lineEditInputFile_textChanged(const QString &text);

    void on_lineEditOutputFile_textEdited(const QString &text);

    void on_lineEditOutputFile_textChanged(const QString &text);

    void on_BrowseInput_clicked();

    void on_BrowseOutput_clicked();


    void on_lineEditLeverArmX_textEdited(const QString &text);

    void on_lineEditLeverArmY_textEdited(const QString &arg1);

    void on_lineEditLeverArmZ_textEdited(const QString &arg1);

private:

    void setStateProcess();

    void possiblyUpdateOutputFileName();

    void setLeverArm( const QString &text, const int position );

    Ui::MainWindow *ui;

    std::string inputFileName;

    std::string outputFileName;

    QString currentInputPath;

    QString currentOutputPath;

    bool outputFileNameEditedByUser;

    // double leverArm[ 3 ];

    Eigen::Vector3d leverArm;

};

#endif // MAINWINDOW_H
