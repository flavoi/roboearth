/** \file mainwindow.cpp
 * \brief Mainwindow implementation
 *
 * This file is part of the RoboEarth ROS re_object_detector_gui package.
 *
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by by <a href="mailto:daniel.dimarco@ipvs.uni-stuttgart.de">Daniel Di Marco</a>, University of Stuttgart
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
 * \author Daniel Di Marco
 * \author Andreas Koch
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <ros/ros.h>

#include <tinyxml.h>

#include <QDir>
#include <QFileDialog>
#include <QTableWidgetItem>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>

#include "comthread.h"
#include "downloadobjectdialog.h"

struct MetaData {
    std::string model_name, model_type;
    size_t face_count;
    double scale;

    bool load(const std::string& filename) {
        TiXmlDocument doc(filename);
        bool loaded = doc.LoadFile();
        if (!loaded)
            return false;

        TiXmlHandle hdoc(&doc);
        TiXmlHandle model_handle = hdoc.FirstChildElement("model");
        TiXmlElement *model_element = model_handle.Element();
        if (model_element == NULL)
            return false;
        TiXmlElement *name_element = model_handle.FirstChildElement("name").Element();
        if (name_element == NULL)
            return false;
        model_name = name_element->GetText();
        TiXmlElement *type_element = model_handle.FirstChildElement("type").Element();
        if (type_element == NULL)
            return false;
        model_type = type_element->GetText();
        TiXmlElement *faces_element = model_handle.FirstChildElement("faces").Element();
        if (faces_element == NULL)
            return false;
        std::string faces_str = faces_element->GetText();
        face_count = boost::lexical_cast<int>(faces_str);

        TiXmlHandle dimensions_handle = model_handle.FirstChildElement("dimensions");
        TiXmlElement *scale_element = dimensions_handle.FirstChildElement("scale").Element();
        if (scale_element == NULL)
            return false;
        std::string scale_str = scale_element->GetText();
        scale = boost::lexical_cast<double>(scale_str);

        return true;
    }
};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), comthread(new ComThread)
{
    ui->setupUi(this);

    connect(this, SIGNAL(model_added(QString,QString,QString)), comthread, SLOT(publishModelDir(QString,QString,QString)));

    connect(comthread, SIGNAL(updateZaragozaDetectionImg(QImage)), this, SLOT(updateZaragozaDetectionImg(QImage)));
    connect(comthread, SIGNAL(updateKinectDetectionImg(QImage)), this, SLOT(updateKinectDetectionImg(QImage)));

    imgLabelZaragoza = new QLabel();
    imgLabelZaragoza->resize(640, 480);

    imgLabelKinect = new QLabel();
    imgLabelKinect->resize(640, 480);

    comthread->start();
}

void MainWindow::closeEvent(QCloseEvent *) {
    imgLabelZaragoza->close();
    imgLabelKinect->close();
}

MainWindow::~MainWindow()
{
    ros::shutdown();
    delete ui;
}

void MainWindow::on_downloadModelButton_clicked()
{
    DownloadObjectDialog dlg(this);
    int result = dlg.exec();

    if (result == QDialog::Rejected)
        return;

    Q_FOREACH(QString model_dir, dlg.getDownloadedModelDirs()) {
        addLocalModel(model_dir);
    }
}

void MainWindow::addLocalModel(QString model_dir_str) {
    if (!model_dirs.contains(model_dir_str)) {
        QDir model_dir(model_dir_str);

        MetaData md;
//        try {
//            md.load(model_dir.filePath("meta.xml").toStdString());
//        } catch (std::exception& e) {
//            QMessageBox::warning(this, "Error opening model", "Could not load model directory:\n\n"+QString::fromStdString(e.what()));
//            return;
//        }
        if (!md.load(model_dir.filePath("meta.xml").toStdString())) {
            QMessageBox::warning(this, "Error opening model", "Could not load model directory. Check the 'meta.xml' file.");
            return;
        }

        QTableWidgetItem *pathItem = new QTableWidgetItem(model_dir_str);
        QTableWidgetItem *nameItem = new QTableWidgetItem(QString::fromStdString(md.model_name));
        QTableWidgetItem *typeItem = new QTableWidgetItem(QString::fromStdString(md.model_type));

        int row = ui->modelsTableWidget->rowCount();
        ui->modelsTableWidget->insertRow(row);
        ui->modelsTableWidget->setItem(row, 0, pathItem);
        ui->modelsTableWidget->setItem(row, 1, nameItem);
        ui->modelsTableWidget->setItem(row, 2, typeItem);

        model_dirs.insert(model_dir_str);
        emit model_added(model_dir_str, QString::fromStdString(md.model_type), QString::fromStdString(md.model_name));
    }
}

void MainWindow::on_addLocalModelButton_clicked()
{
    QString model_dir_str = QFileDialog::getExistingDirectory(this, "Please specify a model directory");

    if (!model_dir_str.isEmpty())
        addLocalModel(model_dir_str);
}

void MainWindow::updateZaragozaDetectionImg(QImage img) {
    if (!imgLabelZaragoza->isVisible())
        imgLabelZaragoza->show();
    imgLabelZaragoza->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::updateKinectDetectionImg(QImage img) {
    if (!imgLabelKinect->isVisible())
        imgLabelKinect->show();
    imgLabelKinect->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::on_resendModelsButton_clicked() {
    for (int i=0; i<ui->modelsTableWidget->rowCount(); i++){
        QTableWidgetItem *pathItem = ui->modelsTableWidget->item(i, 0);
        QTableWidgetItem *nameItem = ui->modelsTableWidget->item(i, 1);
        QTableWidgetItem *typeItem = ui->modelsTableWidget->item(i, 2);

        emit model_added(pathItem->text(), typeItem->text(), nameItem->text());
    }
}
