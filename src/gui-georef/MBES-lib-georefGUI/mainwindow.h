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

    void on_Browse_clicked();

    void on_Process_clicked();

    void on_lineEdit_textChanged(const QString &text);

private:
    Ui::MainWindow *ui;

    std::string inputFileName;
};

#endif // MAINWINDOW_H
