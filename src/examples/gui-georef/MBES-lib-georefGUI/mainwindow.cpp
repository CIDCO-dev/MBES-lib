/*
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#include <QFileDialog>

#include <QMessageBox>

#include <QDebug>

// #include <sstream>

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

    originalLeverArmPointSize{ 12, 12, 12 }, // Temporary possible values
    originalLeverArmPixelSize{ 12, 12, 12 }, // Temporary possible values
    originalLeverArmSpecifiedWithPointSize{ true, true, true, },

    processToolTipTextWhenDisabled( tr( "'Process' button only enabled when there are input and output files" ) )
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
    setStateProcess();

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
    // Disable the button while processing
    ui->Process->setEnabled( false );
    ui->Process->setText( tr( "Processing" ) );
    this->setFocus();
    QCoreApplication::processEvents();


    QFileInfo infoInput( tr( outputFileName.c_str() ) );

    if ( infoInput.exists() )
    {
        std::string absoluteFilePath( infoInput.absoluteFilePath().toLocal8Bit().constData() );
        std::string fileName( infoInput.fileName().toLocal8Bit().constData() );

        QMessageBox msgBox( this );

        std::string text = "A file named \"" + fileName + "\" already exists.\n"
                + "Do you want to replace it?";

        std::string informativeText = "The complete file path/name is \n\n\""
                                            + absoluteFilePath
                                            + "\"\n\nReplacing it will overwrite its contents.";

        msgBox.setText( tr( text.c_str() ) );
        msgBox.setInformativeText( tr( informativeText.c_str() ) );

        msgBox.setStandardButtons( QMessageBox::Cancel | QMessageBox::Ok );
        msgBox.setDefaultButton( QMessageBox::Cancel );

        msgBox.setIcon( QMessageBox::Question );

        int userInput = msgBox.exec();

        if( userInput == QMessageBox::Cancel )
        {
            ui->Process->setText( tr( "Process" ) );
            setStateProcess();

            return;
        }

    }


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


                qDebug() << "Done decoding \n" << tr( inputFileName.c_str() );


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
        std::string toDisplay = "Error while parsing file \n\n\"" + inputFileName + "\":\n\n" + error->getMessage() + ".\n";

        qDebug() << tr( toDisplay.c_str() );

        QMessageBox::warning( this, tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );
    }
    catch ( const char * message )
    {
        std::string toDisplay = "Error while parsing file \n\n\"" + inputFileName + "\":\n\n" + message + ".\n";

        qDebug() << tr( toDisplay.c_str() );

        QMessageBox::warning( this, tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );
    }
    catch (...)
    {
        std::string toDisplay = "Error while parsing file \n\n\"" + inputFileName + "\":\n\nOther exception" + ".\n";

        qDebug() << tr( toDisplay.c_str() );

        QMessageBox::warning( this, tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );
    }


    if(parser)
        delete parser;

    ui->Process->setText( tr( "Process" ) );
    setStateProcess();

}


void MainWindow::setStateProcess()
{
    if( inputFileName != "" && outputFileName != "" )
    {
        ui->Process->setEnabled( true );
        ui->Process->setToolTip( "" );
    }
    else
    {
        ui->Process->setEnabled( false );
        ui->Process->setToolTip( processToolTipTextWhenDisabled );
    }

}



void MainWindow::on_lineEditInputFile_textChanged(const QString &text)
{
    inputFileName = text.toLocal8Bit().constData();

    setStateProcess();

    QFileInfo fileInfo( tr( inputFileName.c_str() ) );

    if ( inputFileName != "" && QDir( fileInfo.absolutePath() ).exists() )
    {
        currentInputPath = fileInfo.absolutePath();
    }
    else
    {
        currentInputPath = "";
    }


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


    QFileInfo fileInfo( tr( outputFileName.c_str() ) );

    if ( outputFileName != "" && QDir( fileInfo.absolutePath() ).exists() )
    {
        currentOutputPath = fileInfo.absolutePath();
    }
    else
    {
        currentOutputPath = "";
    }
}

void MainWindow::on_BrowseInput_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                        tr( "File to Georeference"), currentInputPath,
                                        tr( "*.all *.xtf *.s7k;;*.all;;*.xtf;;*.s7k;;All Files (*)") );

    if ( ! fileName.isEmpty() )
    {
        inputFileName = fileName.toLocal8Bit().constData();

        // Put the file name in the lineEdit
        ui->lineEditInputFile->setText( fileName );

        QFileInfo infoInput( tr( inputFileName.c_str() ) );

        currentInputPath = infoInput.absolutePath();


        // If the user did not edit the output file name himself using the QLineEdit
        if ( outputFileNameEditedByUser == false )
        {
            // Set an output path/file name based on the input file path / name

            std::string absolutePath( infoInput.absolutePath().toLocal8Bit().constData() );
            std::string completeBaseName( infoInput.completeBaseName().toLocal8Bit().constData() );
            outputFileName = absolutePath + "/" + completeBaseName + ".MBES-libGeoref.txt";

            // Put the file name in the lineEdit
            ui->lineEditOutputFile->setText( tr( outputFileName.c_str() ) );

            currentOutputPath = currentInputPath;
        }

        setStateProcess();

    }
}



void MainWindow::on_BrowseOutput_clicked()
{
    QString fileName = QFileDialog::getSaveFileName( this,
                                        tr( "Georeferenced Output File"), currentOutputPath,
                                        tr( "*.txt;;All Files (*)" ), nullptr,
                                                    QFileDialog::DontConfirmOverwrite ) ;

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
        //std::cout << "\nleverArm( position ): "<< leverArm( position ) << std::endl;
    }
    else
    {
        std::string toDisplay = "\"" + std::string( text.toLocal8Bit().constData() ) + "\" is not a valid number\n";

        qDebug() << tr( toDisplay.c_str() );

        QMessageBox::warning( this,tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );
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
                                        tr( "Load Lever Arm Values from File" ), "",
                                        tr( "*.txt;;All Files (*)" )  );

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
                std::string toDisplay = "Problem reading the file \n\n\"" + std::string( fileName.toLocal8Bit().constData() )
                                           +"\".\n\nCould not read three double values for the lever arms.\n";

                qDebug() << tr( toDisplay.c_str() );

                QMessageBox::warning( this,tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );
            }

        }
        else
        {
            std::string toDisplay = "Could not open file \n\n\"" + std::string( fileName.toLocal8Bit().constData() ) +"\".\n";

            qDebug() << tr( toDisplay.c_str() );

            QMessageBox::warning( this,tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );

        }

    }


}


void MainWindow::on_LeverArmSave_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                        tr( "Save Lever Arm Values to File"), "",
                                        tr( "*.txt;;All Files (*)") );

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
            std::string toDisplay = "Could not save to file \n\n\"" + saveFileName + "\".";

            qDebug() << tr( toDisplay.c_str() );

            QMessageBox::warning( this,tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );

        }

    }

}

void MainWindow::on_buttonAbout_clicked()
{
    std::string text = "\n\nCopyright 2017-2019\n"
                        "© Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés\n"
                       "© Interdisciplinary Centre for the Development of Ocean Mapping (CIDCO), All Rights Reserved\n\n";

    QMessageBox::about( this, tr( "About 'MBES-Lib Georeferencing'" ),
                          QString::fromUtf8( text.c_str() )  );

}
