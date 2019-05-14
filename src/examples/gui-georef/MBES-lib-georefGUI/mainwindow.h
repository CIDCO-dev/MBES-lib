/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

 /*
 * \author Christian Bouchard
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


    void on_lineEditOutputFile_textEdited(const QString &text);
    void on_lineEditInputFile_editingFinished();

    void on_lineEditOutputFile_editingFinished();


    void on_BrowseInput_clicked();
    void on_BrowseOutput_clicked();


    void on_lineEditLeverArmX_textEdited(const QString &text);
    void on_lineEditLeverArmY_textEdited(const QString &arg1);
    void on_lineEditLeverArmZ_textEdited(const QString &arg1);

    void on_lineEditRoll_textEdited(const QString &arg1);
    void on_lineEditPitch_textEdited(const QString &arg1);
    void on_lineEditYaw_textEdited(const QString &arg1);


    void on_lineEditLeverArmX_editingFinished();
    void on_lineEditLeverArmY_editingFinished();
    void on_lineEditLeverArmZ_editingFinished();

    void on_lineEditRoll_editingFinished();
    void on_lineEditPitch_editingFinished();
    void on_lineEditYaw_editingFinished();


    void on_LeverArmLoad_clicked();

    void on_LeverArmSave_clicked();


    void on_actionAbout_triggered();

    void on_actionExit_triggered();

    void on_actionLoad_Lever_Arms_and_Boresight_Angles_triggered();

    void on_actionSave_Arms_and_Boresight_Angles_triggered();



private:

    void setStateProcess();

    bool setValueDouble( const QString &text, const int position );

    void editingFinished( const int position );

    void adjustLineEditFontSize( const int position );


    void leverArmBoresightLoad();
    void leverArmBoresightSave();

    std::string removeLeadingTrailingWhitespaces( const std::string &text );

    Ui::MainWindow *ui;

    // File names including the absolute path
    std::string inputFileName;
    std::string outputFileName;

    QString currentInputPath;

    QString currentOutputPath;

    bool outputFileNameEditedByUser;

    static const QString processToolTipTextWhenDisabled;

    static const int nbValuesD = 6; // Number of lineEdit double values
    static const std::string lineEditNames[ nbValuesD ];

    const int lineEditFontPointSizeChange = 2;
    const int lineEditFontPixelSizeChange = 2;

    bool lineEditUserEditing[ nbValuesD ];

    int lineEditOriginalPointSize[ nbValuesD ];
    int lineEditOriginalPixelSize[ nbValuesD ];

    bool lineEditSpecifiedWithPointSize[ nbValuesD ];

    QLineEdit * lineEditPointers[ nbValuesD ];

    Eigen::VectorXd valuesD;

    bool currentlyProcessing;
};

#endif // MAINWINDOW_H
