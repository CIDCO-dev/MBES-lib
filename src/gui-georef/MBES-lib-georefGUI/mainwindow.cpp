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

#include "DatagramGeoreferencerToOstream.hpp"

#include "../../datagrams/kongsberg/KongsbergParser.hpp"
#include "../../datagrams/xtf/XtfParser.hpp"
#include "../../datagrams/s7k/S7kParser.hpp"


#include "../../utils/StringUtils.hpp"
#include "../../utils/Exception.hpp"




MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),

    inputFileName( "" ),
    outputFileName( "" ),

    currentInputPath( "" ),
    currentOutputPath( "" ),

    outputFileNameEditedByUser( false )
{
    ui->setupUi(this);

    ui->lineEditInputFile->setText( tr( inputFileName.c_str() ) );

    ui->lineEditOutputFile->setText( tr( outputFileName.c_str() ) );

    // Disable process button
    ui->Process->setEnabled(false);

    setWindowTitle( tr( "MBES-Lib Georeferencing" ) );
}

MainWindow::~MainWindow()
{
    delete ui;
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

        std::ofstream outFile;
        outFile.open( outputFileName, std::ofstream::out | std::ofstream::trunc );

        if (outFile)
        {
            std::ifstream inFile;
            inFile.open( inputFileName );

            qDebug() << "Decoding \n" << tr( inputFileName.c_str() );

            if (inFile)
            {

                DatagramGeoreferencerToOstream printer( outFile );

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

                parser->parse( inputFileName );

                Eigen::Vector3d leverArm;

                leverArm << leverArmX,leverArmY,leverArmZ;

                printer.georeference(leverArm);


            }
            else
            {
                throw new Exception("Input file not found");
            }

        }
        else
        {
            throw new Exception("Could not open output file");
        }
    }
    catch(Exception * error)
    {

        std::ostringstream streamToDisplay;

        streamToDisplay << "Error while parsing file \n\"" <<inputFileName << "\":\n\n" << error->getMessage() << std::endl;


        qDebug() << tr( streamToDisplay.str().c_str() );

        QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );

    }
    catch ( const char * message )
    {
        std::ostringstream streamToDisplay;

        streamToDisplay << "Error while parsing file \n\"" <<inputFileName << "\":\n\n" << message << std::endl;


        qDebug() << tr( streamToDisplay.str().c_str() );

        QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );
    }
    catch (...)
    {
        std::ostringstream streamToDisplay;

        streamToDisplay << "Error while parsing file \n\"" <<inputFileName << "\":\n\nOther exception" << std::endl;


        qDebug() << tr( streamToDisplay.str().c_str() );

        QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );

    }


    if(parser) delete parser;

}


void MainWindow::setStateProcess()
{
    if( inputFileName != "" && outputFileName != "" )
    {
        ui->Process->setEnabled( true );
    }
    else
    {
        ui->Process->setEnabled( false );
    }

}

void MainWindow::possiblyUpdateOutputFileName()
{

    QFileInfo infoInput( tr( inputFileName.c_str() ) );


    if ( infoInput.exists() )
    {
        currentInputPath = infoInput.absolutePath();

        if ( outputFileNameEditedByUser == false )
        {
            // Set an output path/file name based on the input file path / name

            std::string absolutePath( infoInput.absolutePath() .toLocal8Bit().constData() );
            std::string completeBaseName( infoInput.completeBaseName() .toLocal8Bit().constData() );
            outputFileName = absolutePath + "/" + completeBaseName + ".MBES-libGeoref.txt";

            // Put the file name in the lineEdit
            ui->lineEditOutputFile->setText( tr( outputFileName.c_str() ) );
        }

    }

}


void MainWindow::on_lineEditInputFile_textChanged(const QString &text)
{
    inputFileName = text.toLocal8Bit().constData();

    possiblyUpdateOutputFileName();

    setStateProcess();
}

// https://doc.qt.io/qt-5/qlineedit.html#textEdited
// This function is called when the text is not changed programmatically (that is, when it is edited)
void MainWindow::on_lineEditOutputFile_textEdited(const QString &text)
{
//    std::cout << "\nBeginning of on_lineEditOutputFile_textEdited\n" << std::endl;

    if( text.isEmpty()  )
        outputFileNameEditedByUser = false; // Reset variable
    else
        outputFileNameEditedByUser = true;

}

// This function is called when the text is edited and when it is changed programmatically
void MainWindow::on_lineEditOutputFile_textChanged(const QString &text)
{
//    std::cout << "\nBeginning of on_lineEditOutputFile_textChanged\n" << std::endl;

    outputFileName = text.toLocal8Bit().constData();

    setStateProcess();

    QFileInfo infoOutput( tr( outputFileName.c_str() ) );

    if ( QDir( infoOutput.absolutePath() ).exists() )
    {
        currentOutputPath = infoOutput.absolutePath();
    }
}

void MainWindow::on_BrowseInput_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                        tr( "File to Georeference"), currentInputPath,
                                        tr( "*.all *.xtf *.s7k;;*.all;;*.xtf;;*.s7k;;All Files (*)") );

    if ( ! fileName.isEmpty() )
    {
        std::string OldInputFileName = inputFileName;
        inputFileName = fileName.toLocal8Bit().constData();

        // Put the file name in the lineEdit
        ui->lineEditInputFile->setText( fileName );

        // If the file name does not change, function MainWindow::on_lineEditInputFile_textChanged() will not be called,
        // So do here what would be done by the function
        if ( inputFileName == OldInputFileName )
        {
            possiblyUpdateOutputFileName();

            setStateProcess();
        }
    }
}



void MainWindow::on_BrowseOutput_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                        tr( "Georeferenced Output File"), currentOutputPath);

    if ( ! fileName.isEmpty() )
    {
        outputFileName = fileName.toLocal8Bit().constData();

        // Put the file name in the lineEdit
        ui->lineEditOutputFile->setText( fileName );
    }
}


