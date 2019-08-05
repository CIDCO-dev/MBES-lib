/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

 /*
 * \author Christian Bouchard
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
#include "../../../math/Boresight.hpp"


const std::string MainWindow::lineEditNames[ nbValuesD ] = { "Lever arm X", "Lever arm Y", "Lever arm Z",
                                                                "Roll", "Pitch", "Yaw" };

const QString MainWindow::processToolTipTextWhenDisabled( tr( "'Process' button only enabled when there are input and output files" ) );


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),

    inputFileName( "" ),
    outputFileName( "" ),

    currentInputPath( "" ),
    currentOutputPath( "" ),

    outputFileNameEditedByUser( false ),

    currentlyProcessing( false )
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


    valuesD.conservativeResize( nbValuesD );

    valuesD << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;


    // Try and read from file the last lever arm and boresight values used.
    std::ifstream inFile;
    inFile.open( "lastLeverArmsBoresights.txt" );

    if ( inFile )
    {
        double values[ nbValuesD ];

        for ( int count= 0; count < nbValuesD; count++ )
            inFile >> values[ count ];

        if( inFile.fail() == false )
        {
            for ( int count= 0; count < nbValuesD; count++ )
                valuesD( count ) = values[ count ];
        }
    }


    lineEditPointers[ 0 ] = ui->lineEditLeverArmX;
    lineEditPointers[ 1 ] = ui->lineEditLeverArmY;
    lineEditPointers[ 2 ] = ui->lineEditLeverArmZ;

    lineEditPointers[ 3 ] = ui->lineEditRoll;
    lineEditPointers[ 4 ] = ui->lineEditPitch;
    lineEditPointers[ 5 ] = ui->lineEditYaw;

    for ( int count = 0; count < nbValuesD; count++ )
    {
        lineEditUserEditing[ count ] = false;

        lineEditOriginalPointSize[ count ] = 12; // Temporary possible values
        lineEditOriginalPixelSize[ count ] = 12; // Temporary possible values
        lineEditSpecifiedWithPointSize[ count ] = true;

        lineEditPointers[ count ]->setText( QString::number( valuesD( count ), 'f', 6 ) );
        lineEditPointers[ count ]->setAlignment(Qt::AlignRight);
    }


}

MainWindow::~MainWindow()
{
    // Save to file the last lever arm and boresight values used

    std::ofstream outFile;
    outFile.open( "lastLeverArmsBoresights.txt", std::ofstream::out | std::ofstream::trunc );

    if( outFile )
    {
        outFile << std::setprecision(6) << std::fixed << valuesD << std::endl;
    }

    delete ui;
}


void MainWindow::on_Process_clicked()
{

    currentlyProcessing = true;

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

            currentlyProcessing = false;

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

            qDebug() << "Georeferencing \n" << tr( inputFileName.c_str() );

            if (inFile)
            {
                //TODO: allow selection between georeferencing modes
                Georeferencing * georef = new GeoreferencingTRF();

                DatagramGeoreferencerToOstream printer( outFile ,georef );

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

                Eigen::Vector3d leverArm = valuesD.head( 3 );

//                std::cout << "\n" << valuesD(3)
//                          << "\n" <<valuesD(4)
//                          << "\n" <<valuesD(5) << "\n" << std::endl;


                Attitude boresightAngles( 0, valuesD(3), valuesD(4), valuesD(5) ); //Attitude boresightAngles(0,roll,pitch,heading);
                Eigen::Matrix3d boresight;
                Boresight::buildMatrix( boresight, boresightAngles );

                //TODO: get SVP

                printer.georeference( leverArm, boresight,NULL );

                qDebug() << "Done georeferencing \n" << tr( inputFileName.c_str() );


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

    currentlyProcessing = false;

    setStateProcess();

}


void MainWindow::setStateProcess()
{
    if ( currentlyProcessing == false )
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

}

std::string MainWindow::removeLeadingTrailingWhitespaces( const std::string &text )
{
    const std::string whitespace = " \t";

    const size_t first = text.find_first_not_of( whitespace );

    if ( first == std::string::npos )
        return "";

    const size_t last = text.find_last_not_of( whitespace );

    const size_t length = last - first + 1;

    return text.substr( first, length );

}



// https://doc.qt.io/qt-5/qlineedit.html#textEdited
// This function is called when the text is not changed programmatically
// (that is, the function is called when the text is edited)
void MainWindow::on_lineEditOutputFile_textEdited(const QString &text)
{
//    std::cout << "\nBeginning of on_lineEditOutputFile_textEdited\n" << std::endl;

    if( text.isEmpty()  )
        outputFileNameEditedByUser = false; // Reset variable
    else
        outputFileNameEditedByUser = true;

}



void MainWindow::on_lineEditInputFile_editingFinished()
{
//    std::cout << "\nBeginning of on_lineEditInputFile_editingFinished()\n" << std::endl;

    inputFileName = removeLeadingTrailingWhitespaces( ui->lineEditInputFile->text().toLocal8Bit().constData() );

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


    // If line edit has the focus, set it to the main window
    if ( ui->lineEditInputFile->hasFocus() )
        this->setFocus();

//    std::cout << "\nEnd of on_lineEditInputFile_editingFinished()\n" << std::endl;
}

void MainWindow::on_lineEditOutputFile_editingFinished()
{
    //    std::cout << "\nBeginning of on_lineEditOutputFile_editingFinished()\n" << std::endl;

    outputFileName = removeLeadingTrailingWhitespaces( ui->lineEditOutputFile->text().toLocal8Bit().constData() );

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


    // If line edit has the focus, set it to the main window
    if ( ui->lineEditOutputFile->hasFocus() )
        this->setFocus();
}




void MainWindow::on_BrowseInput_clicked()
{

    QString preSelect = tr( inputFileName .c_str() );

    QFileInfo fileInfoPreDialog( preSelect );

    if ( inputFileName == "" || fileInfoPreDialog.exists() == false  )
    {
        preSelect = currentInputPath;
    }


    QFileDialog dialog( this,
                         tr( "File to Georeference"), preSelect,
                        tr( "*.all *.xtf *.s7k;;*.all;;*.xtf;;*.s7k;;All Files (*)") );

    dialog.setFileMode( QFileDialog::AnyFile ); // Get a single file, whether it exists or not
    dialog.setLabelText( QFileDialog::Accept, tr( "Select" ) ) ; // Name of the button, to replace the default "Open"
    dialog.setViewMode( QFileDialog::Detail );
    dialog.setOptions( QFileDialog::DontConfirmOverwrite );

    QStringList fileNames;

    if ( dialog.exec() )
        fileNames = dialog.selectedFiles();


    if ( fileNames.size() > 0 )
    {
        QString fileName = fileNames.at( 0 );

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
    QString preSelect = tr( outputFileName .c_str() );

    QFileInfo fileInfoPreDialog( preSelect );

    if ( outputFileName == "" || fileInfoPreDialog.exists() == false  )
    {
        std::string fileNameIncludingSuffix( fileInfoPreDialog.fileName().toLocal8Bit().constData() );

        if ( fileNameIncludingSuffix == "" )
            preSelect = currentOutputPath;
        else
        {
            std::string absolutePath( fileInfoPreDialog.absolutePath().toLocal8Bit().constData() );

            std::string name = absolutePath + "/" + fileNameIncludingSuffix;

            preSelect = tr( name.c_str() );
        }
    }


    QFileDialog dialog( this,
                        tr( "Georeferenced Output File"), preSelect,
                        tr( "*.txt;;All Files (*)" ) );

    dialog.setFileMode( QFileDialog::AnyFile ); // Get a single file, whether it exists or not
    dialog.setLabelText( QFileDialog::Accept, tr( "Select" ) ) ; // Name of the button, to replace the default "Open"
    dialog.setViewMode( QFileDialog::Detail );
    dialog.setOptions( QFileDialog::DontConfirmOverwrite );

    QStringList fileNames;
    if ( dialog.exec() )
        fileNames = dialog.selectedFiles();

    if ( fileNames.size() > 0 )
    {
        QString fileName = fileNames.at( 0 );

        outputFileName = fileName.toLocal8Bit().constData();

        // Put the file name in the lineEdit
        ui->lineEditOutputFile->setText( fileName );
    }

    setStateProcess();
}


void MainWindow::adjustLineEditFontSize( const int position )
{
    // If fist time the function is called while the user modifies the value
    if ( lineEditUserEditing[ position ] == false )
    {

        lineEditUserEditing[ position ] = true;

        // Change font size

        QFont font = lineEditPointers[ position ]->font();
        int pointSize = font.pointSize();

//        std::cout << "\npointSize: " << pointSize << "\n" << std::endl;


        // If font specified with point size
        if ( pointSize != -1 )
        {
            lineEditSpecifiedWithPointSize[ position ] = true;

            lineEditOriginalPointSize[ position ] = pointSize;

            font.setPointSize( lineEditOriginalPointSize[ position ] + lineEditFontPointSizeChange );
            lineEditPointers[ position ]->setFont(font);
        }
        else    // Font specified with pixel size;
        {
            lineEditSpecifiedWithPointSize[ position ] = false;

            lineEditOriginalPixelSize[ position ] = font.pixelSize();

            font.setPixelSize( lineEditOriginalPixelSize[ position ] + lineEditFontPixelSizeChange );
            lineEditPointers[ position ]->setFont(font);
        }

    }

}


void MainWindow::on_lineEditLeverArmX_textEdited(const QString &text)
{
    if ( lineEditUserEditing[ 0 ] == false )
    {
        adjustLineEditFontSize( 0 );
    }
}

void MainWindow::on_lineEditLeverArmY_textEdited(const QString &text)
{
    if ( lineEditUserEditing[ 1 ] == false )
    {
        adjustLineEditFontSize( 1 );
    }
}

void MainWindow::on_lineEditLeverArmZ_textEdited(const QString &text)
{
    if ( lineEditUserEditing[ 2 ] == false )
    {
        adjustLineEditFontSize( 2 );
    }
}


void MainWindow::on_lineEditRoll_textEdited(const QString &arg1)
{
    if ( lineEditUserEditing[ 3 ] == false )
    {
        adjustLineEditFontSize( 3 );
    }
}

void MainWindow::on_lineEditPitch_textEdited(const QString &arg1)
{
    if ( lineEditUserEditing[ 4 ] == false )
    {
        adjustLineEditFontSize( 4 );
    }
}

void MainWindow::on_lineEditYaw_textEdited(const QString &arg1)
{
    if ( lineEditUserEditing[ 5 ] == false )
    {
        adjustLineEditFontSize( 5 );
    }
}




bool MainWindow::setValueDouble( const QString &text, const int position )
{

//    std::cout << "\nBeginning of setValueDouble(), text: \"" << text.toLocal8Bit().constData()
//              << "\", position: " << position << std::endl;

    // TODO: how to deal with precision of the double in memory and used for the
    // georeferencing vs. what is displayed in the GUI?

    bool OK = false;

    double value = text.toDouble( &OK );

    if ( OK )
    {
        valuesD( position ) = value;
        //std::cout << "\nvaluesD( position ): "<< valuesD( position ) << std::endl;
    }
    else
    {
        std::string toDisplay = "\"" + std::string( text.toLocal8Bit().constData() ) + "\" is not a valid number for "
                + lineEditNames[ position ];

        qDebug() << tr( toDisplay.c_str() );


        // QT bug: The signal "editingFinished()" is emitted twice when "Enter key" is pressed.
        // This causes two warning dialogs to open.
        // Workaround: block signals to this QLineEdit before opening dialog.
        // https://forum.qt.io/topic/39141/qlineedit-editingfinished-signal-is-emitted-twice/4

        lineEditPointers[ position ]->blockSignals( true );

        QMessageBox::warning( this,tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );

        lineEditPointers[ position ]->blockSignals( false );
    }

    return OK;

}

void MainWindow::editingFinished( const int position )
{

//    std::cout << "\nBeginning of editingFinished(), position: " << position << std::endl;

    if ( lineEditUserEditing[ position ] == true )
    {
        // If line edit has the focus, set it to the main window
        if ( lineEditPointers[ position ]->hasFocus() )
            this->setFocus();

        // Validate if the text in the line edit is a double, if it is: set variable's value
        bool validNumber = setValueDouble( lineEditPointers[ position ]->text(), position );

//        std::cout << "\nvaluesD( position ): " << valuesD( position ) << std::endl;

        if ( validNumber )
        {
            lineEditUserEditing[ position ] = false;

            // Set back the font size
            QFont font = lineEditPointers[ position ]->font();

            // If font specified with point size
            if ( lineEditSpecifiedWithPointSize[ position ] )
            {
                font.setPointSize( lineEditOriginalPointSize[ position ] );
            }
            else    // font specified with pixel size;
            {
                font.setPixelSize( lineEditOriginalPixelSize[ position ] );
            }

            lineEditPointers[ position ]->setFont(font);

            // Set precision of the display
            lineEditPointers[ position ]->setText( QString::number( valuesD( position ), 'f', 6 ) );

            // If line edit has the focus, set it to the main window
            if ( lineEditPointers[ position ]->hasFocus() )
                this->setFocus();

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

void MainWindow::on_lineEditRoll_editingFinished()
{
    editingFinished( 3 );
}

void MainWindow::on_lineEditPitch_editingFinished()
{
    editingFinished( 4 );
}

void MainWindow::on_lineEditYaw_editingFinished()
{
    editingFinished( 5 );
}




void MainWindow::on_LeverArmLoad_clicked()
{
    leverArmBoresightLoad();
}



void MainWindow::leverArmBoresightLoad()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                        tr( "Load Lever Arm and Boresight Values from File" ), "",
                                        tr( "*.txt;;All Files (*)" )  );

    if ( ! fileName.isEmpty() )
    {

        std::ifstream inFile;
        inFile.open( fileName.toLocal8Bit().constData() );

        if ( inFile )
        {
            double values[ nbValuesD ];

            for ( int count= 0; count < nbValuesD; count++ )
                inFile >> values[ count ];

            if( inFile.fail() == false )
            {
                for ( int count = 0; count < nbValuesD; count++ )
                {
                    valuesD( count ) = values[ count ];
                    lineEditPointers[ count ]->setText( QString::number( valuesD( count ), 'f', 6 ) );
                }

            }
            else
            {
                std::ostringstream streamToDisplay;

                streamToDisplay << "Problem reading the file \n\n\"" << fileName.toLocal8Bit().constData()
                                << "\".\n\nCould not read " << nbValuesD << " double values for the lever arms and boresight angles.\n";

                qDebug() << tr( streamToDisplay.str().c_str() );

                QMessageBox::warning( this,tr("Warning"), tr( streamToDisplay.str().c_str() ), QMessageBox::Ok );
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
    leverArmBoresightSave();
}

void MainWindow::leverArmBoresightSave()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                        tr( "Save Lever Arm and Boresight Values to File"), "",
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
            outFile << std::setprecision(6) << std::fixed << valuesD << std::endl;
        }
        else
        {
            std::string toDisplay = "Could not save to file \n\n\"" + saveFileName + "\".";

            qDebug() << tr( toDisplay.c_str() );

            QMessageBox::warning( this,tr("Warning"), tr( toDisplay.c_str() ), QMessageBox::Ok );

        }

    }

}


void MainWindow::on_actionAbout_triggered()
{
    std::string text = "\n\nSave to a text file the point cloud from a multibeam echosounder datagram file.\n\n"
                       "Copyright 2017-2019\n"
                        "© Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés\n"
                       "© Interdisciplinary Centre for the Development of Ocean Mapping (CIDCO), All Rights Reserved\n\n";

    QMessageBox::about( this, tr( "About 'MBES-Lib Georeferencing'" ),
                          QString::fromUtf8( text.c_str() )  );
}

void MainWindow::on_actionExit_triggered()
{
    QCoreApplication::quit();
}

void MainWindow::on_actionLoad_Lever_Arms_and_Boresight_Angles_triggered()
{
    leverArmBoresightLoad();
}

void MainWindow::on_actionSave_Arms_and_Boresight_Angles_triggered()
{
    leverArmBoresightSave();
}
