/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#include <QFileDialog>

#include <QMessageBox>

#include <QDebug>

#include <sstream>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "DatagramGeoreferencerToOstream.hpp"

#include "../../../datagrams/kongsberg/KongsbergParser.hpp"
#include "../../../datagrams/xtf/XtfParser.hpp"
#include "../../../datagrams/s7k/S7kParser.hpp"


#include "../../../utils/StringUtils.hpp"
#include "../../../utils/Exception.hpp"




MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),

    inputFileName( "" ),
    outputFileName( "" ),

    currentInputPath( "" ),
    currentOutputPath( "" ),

    outputFileNameEditedByUser( false ),

    editingLeverArm{ false, false, false },

    originalLeverArmPointSize{ 12, 12, 12}, // Temporary possible values
    originalLeverArmPixelSize{ 12, 12, 12}, // Temporary possible values
    originalLeverArmSpecifiedWithPointSize{ true, true, true, }
{

#ifdef __GNU__
    setenv("TZ", "UTC", 1);
#endif
#ifdef _WIN32
    putenv("TZ");
#endif

    ui->setupUi(this);

    ui->lineEditInputFile->setText( tr( inputFileName.c_str() ) );

    ui->lineEditOutputFile->setText( tr( outputFileName.c_str() ) );


    // Disable process button
    ui->Process->setEnabled(false);

    setWindowTitle( tr( "MBES-Lib Georeferencing" ) );


    leverArm << 0.0, 0.0, 0.0;

    // Try and read from file the last lever arm values used.
    std::ifstream inFile;
    inFile.open( "lastLeverArms.txt" );

    if ( inFile )
    {
        double values[ 3 ];

        for ( int count= 0; count < 3; count++ )
            inFile >> values[ count];

        if( inFile.fail() == false )
        {
            leverArm << values[ 0 ], values[ 1 ], values[ 2 ];
        }

    }


    lineEditLeverArms[ 0 ] = ui->lineEditLeverArmX;
    lineEditLeverArms[ 1 ] = ui->lineEditLeverArmY;
    lineEditLeverArms[ 2 ] = ui->lineEditLeverArmZ;


    for ( int count = 0; count < 3; count++ )
    {
        lineEditLeverArms[ count ]->setText( QString::number( leverArm( count ), 'f', 6 ) );
        lineEditLeverArms[ count ]->setValidator( new QDoubleValidator( lineEditLeverArms[ count ] ) );
        lineEditLeverArms[ count ]->setAlignment(Qt::AlignRight);
    }


}

MainWindow::~MainWindow()
{
    // Save to file the last lever arm values used

    std::ofstream outFile;
    outFile.open( "lastLeverArms.txt", std::ofstream::out | std::ofstream::trunc );

    if( outFile )
    {
        outFile << std::setprecision(6) << std::fixed
                << leverArm( 0 ) << "\n" << leverArm( 1 ) << "\n" << leverArm( 2 ) << std::endl;
    }


    delete ui;
}


void MainWindow::on_Process_clicked()
{

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

                printer.georeference(leverArm);

                // TODO: ? Display a dialog indicating that the processing is finished?


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

            std::string absolutePath( infoInput.absolutePath().toLocal8Bit().constData() );
            std::string completeBaseName( infoInput.completeBaseName().toLocal8Bit().constData() );
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
                                        tr( "Georeferenced Output File"), currentOutputPath );

    if ( ! fileName.isEmpty() )
    {
        outputFileName = fileName.toLocal8Bit().constData();

        // Put the file name in the lineEdit
        ui->lineEditOutputFile->setText( fileName );
    }
}


void MainWindow::adjustLineEditFontSize( const int position )
{
    // If fist time the function is called while the user modifies the value
    if ( editingLeverArm[ position ] == false )
    {

        editingLeverArm[ position ] = true;

        // Change font size

        QFont font = lineEditLeverArms[ position ]->font();
        int leverArmPointSize = font.pointSize();

//        std::cout << "\nleverArmPointSize: " << leverArmPointSize << "\n" << std::endl;


        // If font specified with point size
        if ( leverArmPointSize != -1 )
        {
            originalLeverArmSpecifiedWithPointSize[ position ] = true;

            originalLeverArmPointSize[ position ] = leverArmPointSize;

            font.setPointSize( originalLeverArmPointSize[ position ] + lineEditleverArmFontPointSizeChange );
            lineEditLeverArms[ position ]->setFont(font);
        }
        else    // Font specified with pixel size;
        {
            originalLeverArmSpecifiedWithPointSize[ position ] = false;

            originalLeverArmPixelSize[ position ] = font.pixelSize();

            font.setPixelSize( originalLeverArmPixelSize[ position ] + lineEditleverArmFontPixelSizeChange );
            lineEditLeverArms[ position ]->setFont(font);
        }

    }

}


void MainWindow::on_lineEditLeverArmX_textEdited(const QString &text)
{
    if ( editingLeverArm[ 0 ] == false )
    {
        adjustLineEditFontSize( 0 );
    }
}

void MainWindow::on_lineEditLeverArmY_textEdited(const QString &text)
{
    if ( editingLeverArm[ 1 ] == false )
    {
        adjustLineEditFontSize( 1 );
    }
}

void MainWindow::on_lineEditLeverArmZ_textEdited(const QString &text)
{
    if ( editingLeverArm[ 2 ] == false )
    {
        adjustLineEditFontSize( 2 );
    }
}


bool MainWindow::setLeverArm( const QString &text, const int position )
{

//    std::cout << "\nBeginning of setLeverArm(), text: \"" << text.toLocal8Bit().constData()
//              << "\", position: " << position << std::endl;

    // TODO: how to deal with precision of the double in memory and used for the
    // georeferencing vs. what is displayed in the GUI?

    bool OK = false;

    double value = text.toDouble( &OK );

    if ( OK )
    {
        leverArm( position ) = value;
    }
    else
    {
        std::ostringstream streamToDisplay;
        streamToDisplay << "\"" << text.toLocal8Bit().constData() << "\" is not a valid number\n";

        qDebug() << tr( streamToDisplay.str().c_str() );

        QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );
    }

    return OK;

}

void MainWindow::editingFinished( const int position )
{

    if ( editingLeverArm[ position ] == true )
    {

        // Validate if the text is the line edit is a double, set lever arm value if so

        bool validNumber = setLeverArm( lineEditLeverArms[ position ]->text(), position );

//        std::cout << "\nleverArm( position ): " << leverArm( position ) << std::endl;

        if ( validNumber )
        {
            editingLeverArm[ position ] = false;

            // Set back the font size
            QFont font = lineEditLeverArms[ position ]->font();

            // If font specified with point size
            if ( originalLeverArmSpecifiedWithPointSize[ position ] )
            {
                font.setPointSize( originalLeverArmPointSize[ position ] );
            }
            else    // font specified with pixel size;
            {
                font.setPixelSize( originalLeverArmPixelSize[ position ] );
            }

            lineEditLeverArms[ position ]->setFont(font);

            // Set precision of the display
            lineEditLeverArms[ position ]->setText( QString::number( leverArm( position ), 'f', 6 ) );

            // If line edit has the focus, set it to the main window
            if ( lineEditLeverArms[ position ]->hasFocus() )
                this->setFocus();

        }
        else
        {
            lineEditLeverArms[ position ]->setFocus();
        }

    }

}



void MainWindow::on_lineEditLeverArmX_editingFinished()
{
//    std::cout << "\nBeginning of on_lineEditLeverArmX_editingFinished()\n" << std::endl;

    editingFinished( 0 );
}


void MainWindow::on_lineEditLeverArmY_editingFinished()
{
    editingFinished( 1 );
}

void MainWindow::on_lineEditLeverArmZ_editingFinished()
{
    editingFinished( 2 );
}






void MainWindow::on_LeverArmLoad_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                        tr( "Load Lever Arm Values from File"), "",
                                        tr( "Text files (*.txt);;All Files (*)")  );

    if ( ! fileName.isEmpty() )
    {

        std::ifstream inFile;
        inFile.open( fileName.toLocal8Bit().constData() );

        if ( inFile )
        {
            double values[ 3 ];

            for ( int count= 0; count < 3; count++ )
                inFile >> values[ count];

            if( inFile.fail() == false )
            {
                leverArm << values[ 0 ], values[ 1 ], values[ 2 ];

                for ( int count = 0; count < 3; count++ )
                    lineEditLeverArms[ count ]->setText( QString::number( leverArm( count ), 'f', 6 ) );

            }
            else
            {
                std::ostringstream streamToDisplay;
                streamToDisplay << "Problem reading the file \n\n\"" << fileName.toLocal8Bit().constData()
                                   << "\"\n\nCould not read three double values for the lever arms" << std::endl;

                qDebug() << tr( streamToDisplay.str().c_str() );

                QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );
            }

        }
        else
        {
            std::ostringstream streamToDisplay;
            streamToDisplay << "Could not open file \n\n\"" << fileName.toLocal8Bit().constData()
                               << "\"" << std::endl;

            qDebug() << tr( streamToDisplay.str().c_str() );

            QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );

        }

    }


}


void MainWindow::on_LeverArmSave_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                        tr( "Save Lever Arm Values to File"), "",
                                        tr( "Text files (*.txt);;All Files (*)") );

    if ( ! fileName.isEmpty() )
    {
        QFileInfo infoInput( fileName );

        const std::string absoluteFilePath( infoInput.absoluteFilePath().toLocal8Bit().constData() );
        const std::string suffix( infoInput.suffix().toLocal8Bit().constData() );

        std::string saveFileName;

        if ( suffix == "txt" )
        {
            saveFileName = absoluteFilePath;
        }
        else
        {
            saveFileName = absoluteFilePath + ".txt";
        }


        std::ofstream outFile;
        outFile.open( saveFileName, std::ofstream::out | std::ofstream::trunc );

        if( outFile )
        {
            outFile << std::setprecision(6) << std::fixed
                    << leverArm( 0 ) << "\n" << leverArm( 1 ) << "\n" << leverArm( 2 ) << std::endl;
        }
        else
        {

            std::ostringstream streamToDisplay;
            streamToDisplay << "Could not save to file \n\n\"" << saveFileName
                               << "\"" << std::endl;

            qDebug() << tr( streamToDisplay.str().c_str() );

            QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );

        }

    }

}
