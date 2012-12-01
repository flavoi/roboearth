/** \file mainwindow.h
 * \brief Implementation of the main user interface
 *
 * This file is part of the RoboEarth ROS re_object_recorder package.
 *
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by by <a href="mailto:andreas.koch@ipvs.uni-stuttgart.de">Andreas Koch</a>, University of Stuttgart
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    <UL>
 *     <LI> Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     <LI> Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     <LI> Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *    </UL>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Andreas Koch
 * \author Daniel Di Marco
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QDir>

namespace Ui {
    class MainWindow;
}

class ComThread;

/**
 * Main user interface.
 **/
class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:
    /**
     * @see ComThread::addStatusMessage
     **/
    void addStatusMessage(QString msg);
    /**
     * @see ComThread::newCloudReceived
     **/
    void insertCloud(QString);

private Q_SLOTS:
    void on_boundingBoxSizeDoubleSpinBox_valueChanged(double );
    void on_cloudListWidget_itemSelectionChanged();
    void on_saveButton_clicked();
    void on_reButton_clicked();
    void on_objectClassEdit_textChanged(const QString &arg1);
    void on_objectNameEdit_textChanged(const QString &arg1);
    void on_apiKeyEdit_textChanged(QString );

    void on_startstopbutton_toggled(bool checked);

    void on_loadButton_clicked();

    void on_actionDelete_triggered();

Q_SIGNALS:
    /**
     * Signals that the bounding box value was changed.
     * @see ComThread::changedBoxSize
     **/
    void changedBoxSize(double);

protected:
    void closeEvent(QCloseEvent *);

    bool checkInput();
    bool saveModels(QString parent_directory_str, QStringList *modelFiles=NULL);

private:
    void updateOWL();
    bool createZippedModel(QDir model_dir, const QString& zip_filename);

    Ui::MainWindow *ui;
    ComThread * comthread;
};

#endif // MAINWINDOW_H
