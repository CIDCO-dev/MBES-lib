/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#include <QFileDialog>
#include <QLineEdit>
#include <QMessageBox>

#include <QDebug>

#include <sstream>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "DatagramGeoreferencerForGUI.hpp"

#include "../../datagrams/kongsberg/KongsbergParser.hpp"
#include "../../datagrams/xtf/XtfParser.hpp"
#include "../../datagrams/s7k/S7kParser.hpp"


#include "../../utils/StringUtils.hpp"
#include "../../utils/Exception.hpp"




MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    inputFileName( "" )
{
    ui->setupUi(this);

    ui->lineEdit->setText( "" );

    // Disable process button
    ui->Process->setEnabled(false);

    setWindowTitle( tr( "MBES-Lib Georeferencing" ) );
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Browse_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                        tr( "File to Georeference"), "",
                                        tr( "*.all *.xtf *.s7k;;*.all;;*.xtf;;*.s7k;;All Files (*)") );

    if ( ! fileName.isEmpty() )
    {
        inputFileName = fileName.toLocal8Bit().constData();

        // Put the file name in the lineEdit
        ui->lineEdit->setText( fileName );
    }
}

void MainWindow::on_Process_clicked()
{

    // TODO: add the possibility for the user to enter the leverArm


    double leverArmX = 0.0;
    double leverArmY = 0.0;
    double leverArmZ = 0.0;



#ifdef __GNU__
    setenv("TZ", "UTC", 1);
#endif
#ifdef _WIN32
    putenv("TZ");
#endif

    DatagramParser * parser = nullptr;





    try
    {

        // TODO: try and open the output file

        // TODO pass the output file stream when creating the DatagramGeoreferencerForGui object

        DatagramGeoreferencerForGui  printer;


        std::ifstream inFile;
        inFile.open( inputFileName );

        qDebug() << "Decoding \n" << tr( inputFileName.c_str() );

        if (inFile)
        {
            if ( ends_with( inputFileName.c_str(),".all" ) )
            {
                parser = new KongsbergParser(printer);
            }
            else if ( ends_with( inputFileName.c_str(),".xtf") )
            {
                parser = new XtfParser(printer);
            }
            else if ( ends_with( inputFileName.c_str(),".s7k") )
            {
                parser = new S7kParser(printer);
            }
            else
            {
                throw new Exception("Unknown extension");
            }
        }
        else
        {
            throw new Exception("Input file not found");
        }
        parser->parse( inputFileName );

        inFile.close();



        Eigen::Vector3d leverArm;

        leverArm << leverArmX,leverArmY,leverArmZ;

        printer.georeference(leverArm);
    }
    catch(Exception * error)
    {

        std::ostringstream streamToDisplay;

        streamToDisplay << "Error while parsing file \n\"" <<inputFileName << "\":\n\n" << error->getMessage() << std::endl;


        qDebug() << tr( streamToDisplay.str().c_str() );

        QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );



        // TODO: Handle "Could not open output file"

//        if ( error->getMessage() == "Could not open output file" )
//        {


//        }

        // TODO: Handle "Unknown extension"

        // TODO: Handle "Input file not found"


    }
    if(parser) delete parser;









//    // TODO: Process the file with MBES-lib

//    // TODO: Handle exception of not being able to open the file

//    // TODO: Handle other exceptions

//    // Temporary:
//    // Try to open the file

//    FILE * file = fopen( inputFileName.c_str(), "rb" );

//    if ( ! file )
//    {
//        qDebug() << "Was NOT able to open the file \n" << tr( inputFileName.c_str() );

//        std::string message = "Could not open file \n\"" + inputFileName + "\"";

//        QMessageBox::warning( this,tr("Warning"), tr( message.c_str() ), QMessageBox::Ok );


//    }
//    else
//    {
//        fclose(file);

//        qDebug() << "Was able to open the file \n" << tr( inputFileName.c_str() );


//    }

}

void MainWindow::on_lineEdit_textChanged(const QString &text)
{
    if ( text == "" )
    {
        ui->Process->setEnabled( false );
    }
    else
    {
        ui->Process->setEnabled( true );
    }

    inputFileName = text.toLocal8Bit().constData();

}
