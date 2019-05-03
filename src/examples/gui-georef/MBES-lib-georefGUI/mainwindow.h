/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QLineEdit>

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

    void on_Process_clicked();


    void on_lineEditInputFile_textChanged(const QString &text);


    void on_lineEditOutputFile_textEdited(const QString &text);

    void on_lineEditOutputFile_textChanged(const QString &text);


    void on_BrowseInput_clicked();

    void on_BrowseOutput_clicked();


    void on_lineEditLeverArmX_textEdited(const QString &text);

    void on_lineEditLeverArmY_textEdited(const QString &arg1);

    void on_lineEditLeverArmZ_textEdited(const QString &arg1);


    void on_lineEditLeverArmX_editingFinished();

    void on_lineEditLeverArmY_editingFinished();

    void on_lineEditLeverArmZ_editingFinished();



    void on_LeverArmLoad_clicked();

    void on_LeverArmSave_clicked();

    void on_buttonAbout_clicked();

private:

    void setStateProcess();

    bool setLeverArm( const QString &text, const int position );

    void editingFinished( const int position );

    void adjustLineEditFontSize( const int position );


    Ui::MainWindow *ui;

    std::string inputFileName;

    std::string outputFileName;

    QString currentInputPath;

    QString currentOutputPath;

    bool outputFileNameEditedByUser;


    const int lineEditleverArmFontPointSizeChange = 2;
    const int lineEditleverArmFontPixelSizeChange = 2;

    bool editingLeverArm[ 3 ];

    int originalLeverArmPointSize[ 3 ];
    int originalLeverArmPixelSize[ 3 ];

    bool originalLeverArmSpecifiedWithPointSize[ 3 ];

    QLineEdit * lineEditLeverArms[ 3 ];

    Eigen::Vector3d leverArm;

    Eigen::Vector3d boresightRPH; // boresight roll, pitch, heading



    QString processToolTipTextWhenDisabled;


};

#endif // MAINWINDOW_H
