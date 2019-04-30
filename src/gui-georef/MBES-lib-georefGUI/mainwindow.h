/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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



private:

    void setStateProcess();

    void possiblyUpdateOutputFileName();

    Ui::MainWindow *ui;

    std::string inputFileName;

    std::string outputFileName;

    QString currentInputPath;

    QString currentOutputPath;

    bool outputFileNameEditedByUser;
};

#endif // MAINWINDOW_H
