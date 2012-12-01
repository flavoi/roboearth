/** \file mainwindow.cpp
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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "comthread.h"
#include <ros/ros.h>
#include <QListWidget>
#include <QKeyEvent>
#include <ros/service_client.h>
#include <re_srvs/SetObject.h>

#include <QtCore>
#include <QProgressDialog>
#include <QFutureWatcher>
#include <QFileDialog>
#include <QTemporaryFile>
#include <QDebug>
#include <QMessageBox>
#include <QDomDocument>

#include <cstdio>

#include "owldescriptioncreator.h"
#include "re_object_recorder/qminizip.h"
#include "uploadhelpers.h"

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    double boxsize_initial = 0.135;
    ui->boundingBoxSizeDoubleSpinBox->setValue(boxsize_initial);
    this->addAction(ui->actionDelete);
    ui->actionDelete->setShortcut(QKeySequence::Delete);

    comthread = new ComThread;
    int maxScans = ui->maxScansSpinBox->value();
    comthread->maxScanNumberChanged(maxScans);

    connect(ui->startstopbutton, SIGNAL(toggled(bool)), comthread, SLOT(recording(bool)));
    connect(ui->resetButton, SIGNAL(clicked()), comthread, SLOT(reset()));
    connect(ui->maxScansSpinBox, SIGNAL(valueChanged(int)), comthread, SLOT(maxScanNumberChanged(int)));
    connect(comthread, SIGNAL(addStatusMessage(QString)), this, SLOT(addStatusMessage(QString)));
    connect(comthread, SIGNAL(newCloudReceived(QString)), this, SLOT(insertCloud(QString)));
    connect(comthread, SIGNAL(finished()), this, SLOT(close()));
    connect(this, SIGNAL(changedBoxSize(double)), comthread, SLOT(changedBoxSize(double)));

    Q_EMIT changedBoxSize(boxsize_initial);

    comthread->start();
    updateOWL();

    bool depth_registration_enabled = false;
    ros::param::get("/camera/driver/depth_registration", depth_registration_enabled);
    if (!depth_registration_enabled) {
        QMessageBox::warning(this, "Depth registration", "Depth registration is disabled in the openni driver."
                             " This will probably lead to badly aligned pointclouds.");
        addStatusMessage("WARNING: Depth registration is disabled in the openni driver.");
    }
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *) {
    ros::shutdown();
    comthread->wait();
}

void MainWindow::addStatusMessage(QString msg) {
    ui->textEdit->append(msg);
}

void MainWindow::insertCloud(QString id) {
    if (id.startsWith("-")) {
        while (ui->cloudListWidget->count()) {
            QListWidgetItem* item = ui->cloudListWidget->takeItem(0);
            if (item!=NULL)
                delete item;
        }
    } else
        ui->cloudListWidget->addItem(id);
}

void MainWindow::on_cloudListWidget_itemSelectionChanged()
{
    QStringList clouds;
    QList<QListWidgetItem*> items = ui->cloudListWidget->selectedItems();
    Q_FOREACH(QListWidgetItem* item, items)
        clouds.append(item->text());
    comthread->selectCloud(clouds);
}

bool MainWindow::createZippedModel(QDir model_dir, const QString& zip_filename)
{
    bool success = true;
    QMiniZip mz(zip_filename, QMiniZip::ZIP_CREATE);

    // add all files in model_dir to zip file
    QDir dir = model_dir;
    dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);

    QFileInfoList list = dir.entryInfoList();
    Q_FOREACH(QFileInfo fi, list) {
        try {
            mz.addFile(fi, model_dir.dirName() + "/" + fi.fileName());
        } catch(const std::exception &e) {
            ui->textEdit->append("Error adding " + fi.fileName() + " to " + zip_filename + " (" + QString(e.what()) + ")");
            success = false;
            break;
        }
    }
    return success;
}

void MainWindow::on_saveButton_clicked()
{
    if (ui->startstopbutton->isChecked())
        ui->startstopbutton->click();

    if (!checkInput())
        return;

    QString parent_directory_str = QFileDialog::getExistingDirectory(this, "Select directory to store the point cloud", QString());

    if (parent_directory_str.isEmpty())
        return;

    QProgressDialog dlg(this);
    dlg.setCancelButton(NULL);
    dlg.setLabelText("Saving model");
    dlg.setWindowModality(Qt::WindowModal);
    dlg.setMinimumDuration(0);
    dlg.setMinimum(0);
    dlg.setMaximum(0);
    dlg.setCancelButton(NULL);

    QFutureWatcher<bool> futureWatcher;
    QObject::connect(&futureWatcher, SIGNAL(finished()), &dlg, SLOT(reset()));
    QObject::connect(&dlg, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));

    QFuture<bool> future = QtConcurrent::run(this, &MainWindow::saveModels, parent_directory_str, static_cast<QStringList*>(0));
    futureWatcher.setFuture(future);

    // Display the dialog and start the event loop.
    dlg.exec();

    futureWatcher.waitForFinished();

    if (!futureWatcher.result()) {
        addStatusMessage("Saving model to '" + parent_directory_str + "'' failed.");
        QMessageBox::warning(this, "Error", "Could not save model");
    }
}

bool MainWindow::checkInput() {
    QString modelname = ui->objectNameEdit->text();
    static const QString msg = "Please specify name and class of the object prior to saving or uploading objects.";
    if (modelname == "") {
        QMessageBox::warning(this, "Insufficient Information", msg);
        ui->textEdit->append(msg);
        ui->tabWidget->setCurrentWidget(ui->uploadTab);
        ui->objectNameEdit->setFocus();
        return false;
    }

    if (modelname.contains(" ")) {
        QMessageBox::warning(this, "Invalid model name", "Model names should not contain spaces.");
        ui->tabWidget->setCurrentWidget(ui->uploadTab);
        ui->objectNameEdit->setFocus();
        return false;
    }
    QString classname = ui->objectClassEdit->text();
    if (classname == "") {
        ui->textEdit->append(msg);
        ui->tabWidget->setCurrentWidget(ui->uploadTab);
        ui->objectClassEdit->setFocus();
        return false;
    }
    if (classname.contains(" ")) {
        QMessageBox::warning(this, "Invalid class name", "Class names should not contain spaces.");
        ui->tabWidget->setCurrentWidget(ui->uploadTab);
        ui->objectClassEdit->setFocus();
        return false;
    }
    return true;
}

bool MainWindow::saveModels(QString parent_directory_str, QStringList *modelFiles) {
    if (modelFiles != NULL)
        modelFiles->clear();

    QString classname = ui->objectClassEdit->text();
    QString modelname = ui->objectNameEdit->text();

    QDir parent_dir(parent_directory_str);

    try {
        parent_dir.mkdir(classname + "." + modelname);
        QDir model_dir = parent_dir;
        model_dir.cd(classname + "." + modelname);
        comthread->createRe_visionModel(model_dir, classname + "." + modelname);
        QString model_filename(parent_dir.absoluteFilePath("2dcammodel.zip"));
        bool success = createZippedModel(model_dir, model_filename);
        if (!success)
            return false;
        if (modelFiles != NULL)
            modelFiles->append(model_filename);
    } catch(const std::string& exception) {
        std::cerr << "caught exception: " << exception << std::endl;
        return false;
    } catch(...) {
        std::cerr << "caught exception" << std::endl;
        return false;
    }

    try {
        parent_dir.mkdir(classname + "." + modelname + "_kinect");
        QDir model_dir = parent_dir;
        model_dir.cd(classname + "." + modelname + "_kinect");
        comthread->createKinectModel(model_dir, classname + "." + modelname);
        QString model_filename(parent_dir.absoluteFilePath("kinectmodel.zip"));
        bool success = createZippedModel(model_dir, model_filename);
        if (!success)
            return false;
        if (modelFiles != NULL)
            modelFiles->append(model_filename);
    } catch(...) {
        std::cerr << "caught exception" << std::endl;
        return false;
    }

    // store preview
    {
        bool created_preview = comthread->createPreview(parent_dir);
        if (created_preview && (modelFiles != NULL))
            modelFiles->append(parent_dir.filePath("preview.png"));
    }

    return true;
}

///// helper function to run the upload service
enum UploadResults {SUCCESS, NO_SERVICE, CHECK_FAILED, GENERIC_UPLOAD_FAILED};
UploadResults uploadModels(UploadBase* uploadGeneric) {
    if (!uploadGeneric->waitForService())
        return NO_SERVICE;
    if (!uploadGeneric->check())
        return CHECK_FAILED;

    if (!uploadGeneric->upload())
        return GENERIC_UPLOAD_FAILED;
    return SUCCESS;
}

void MainWindow::on_reButton_clicked()
{
    if (!checkInput())
        return;
    QString modelname = ui->objectNameEdit->text();
    QString classname = ui->objectClassEdit->text();

    // save temporary models under /tmp/object_recorder_X
    QDir tempDir = QDir::tempPath();
    QString subdir_base = "object_recorder";
    int i=0;
    QString subdir_name = subdir_base + "_" + QString::number(i);
    while (tempDir.exists(subdir_name)) {
        i++;
        subdir_name = subdir_base + "_" + QString::number(i);
    }

    tempDir.mkdir(subdir_name);
    QStringList model_files;
    {
        QProgressDialog dlg("Storing models...", "Cancel", 0, 0, this);
        dlg.setCancelButton(NULL);
        QFutureWatcher<bool> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), &dlg, SLOT(reset()));
        QObject::connect(&dlg, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        // NOTE: QtConcurrent::run creates copies of its arguments before calling the function
        QFuture<bool> future = QtConcurrent::run(this, &MainWindow::saveModels, tempDir.filePath(subdir_name), &model_files);
        futureWatcher.setFuture(future);

        // Display the dialog and start the event loop.
        dlg.exec();

        futureWatcher.waitForFinished();
        if (dlg.wasCanceled()) {
            addStatusMessage("Storing model files was cancelled by user");
            return;
        } else if (!future.result()) {
            qDebug() << "result = false";
            tempDir.rmdir(subdir_name);
            addStatusMessage("Could not store models");
            return;
        }
    }

    // TODO: could we run into memory problems here?
    std::vector<re_msgs::File> fileMsgs;
    Q_FOREACH(const QString& model_filename, model_files) {
        re_msgs::File fileMsg;
        QString actualFilename = model_filename.split('/', QString::SkipEmptyParts).last();
        fileMsg.name = actualFilename.toStdString();
        QFile f(model_filename);
        if (!f.open(QIODevice::ReadOnly)) {
            addStatusMessage("Could not read model file '" + model_filename + "'.");
            continue;
        }

        QByteArray data = f.readAll();
        fileMsg.data.resize(data.size());
        std::copy(data.data(), data.data()+data.size(), fileMsg.data.begin());

        fileMsgs.push_back(fileMsg);
    }

    UploadSetNewObject upload;
    upload.fillRequest(ui->apiKeyEdit->text().toStdString(), modelname.toStdString(), classname.toStdString(), ui->owlView->toPlainText().toStdString(),
                       ui->objectDescriptionEdit->document()->toPlainText().toStdString(), fileMsgs);

    QProgressDialog dlg(this);
    dlg.setCancelButton(NULL);
    dlg.setLabelText("Uploading models to RoboEarth...");
    dlg.setWindowModality(Qt::WindowModal);
    dlg.setMinimumDuration(0);
    dlg.setMinimum(0);
    dlg.setMaximum(0);
    dlg.setCancelButton(NULL);

    QFutureWatcher<UploadResults> futureWatcher;
    QObject::connect(&futureWatcher, SIGNAL(finished()), &dlg, SLOT(reset()));
    QObject::connect(&dlg, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));

    QFuture<UploadResults> future = QtConcurrent::run(&uploadModels, &upload);
    futureWatcher.setFuture(future);

    dlg.exec();
    futureWatcher.waitForFinished();

    UploadResults status = future.result();
    if (status == SUCCESS) {
        addStatusMessage("Successfully transmitted one object to roboearth database.");
    }  else if (status == CHECK_FAILED) {
        static const QString msg = "Upload failed: A model of the same name is already in the database.";
        QMessageBox::warning(this, "Upload failed", msg);
        addStatusMessage(msg);
    } else if (status == NO_SERVICE) {
        static const QString msg = "Upload failed: No service found. Please make sure the re_comm is running.";
        QMessageBox::warning(this, "Upload failed", msg);
        addStatusMessage(msg);
    } else {
        static const QString msg = "Upload failed: Check re_comm output for more information. (Hint: Is your API key correct?)";
        QMessageBox::warning(this, "Upload failed", msg);
        addStatusMessage(msg);
    }
}

void MainWindow::updateOWL() {
    QString model = ui->modelType->text();
    QString objectclass = ui->objectClassEdit->text();
    QString objectname = ui->objectNameEdit->text();

    if (model.isEmpty() || objectclass.isEmpty() || objectname.isEmpty()) {
        ui->reButton->setEnabled(false);
        ui->owlView->setPlainText("");
        return;
    }

    if (!ui->apiKeyEdit->text().isEmpty())
        ui->reButton->setEnabled(true);
    else
        ui->reButton->setEnabled(false);
    OWLDescriptionCreator odc(model, objectclass, QDateTime::currentDateTime());
    ui->owlView->setPlainText(odc.getOwlDescription());
}

void MainWindow::on_objectClassEdit_textChanged(const QString &arg1) {
    updateOWL();
}

void MainWindow::on_objectNameEdit_textChanged(const QString &arg1) {
    updateOWL();
}

void MainWindow::on_boundingBoxSizeDoubleSpinBox_valueChanged(double val) {
    ros::param::set("boxsize", val);
    Q_EMIT changedBoxSize(val);
}

void MainWindow::on_apiKeyEdit_textChanged(QString ) {
    updateOWL();
}


void MainWindow::on_startstopbutton_toggled(bool checked) {
    if (checked) {
        ui->startstopbutton->setText("Stop Recording");
    } else {
        ui->startstopbutton->setText("Start Recording");
    }
}

void MainWindow::on_loadButton_clicked() {
//    QStringList filenames = QFileDialog::getOpenFileNames(this, "Import point clouds", QString(),
//                                                          "PCD point cloud files (*.pcd);;"
//                                                          "Stored Models (*.zip)");
//    Q_FOREACH(const QString& fn, filenames) {
//        comthread->loadPointCloud(fn);
//    }
    comthread->reset();

    QString filename = QFileDialog::getOpenFileName(this, "Import stored model", QString(),
                                                    "Stored Model (*.zip)");
    if (filename.isEmpty())
        return;

    QMiniZip mz(filename, QMiniZip::UNZIP);
    QStringList extracted_files = mz.unzip(QDir::tempPath());
    QString metaFileName;
    Q_FOREACH(const QString& ef, extracted_files) {
        if (ef.toLower().endsWith("meta.xml")) {
            metaFileName = ef;
            break;
        }
    }
    if (metaFileName.isEmpty()) {
        addStatusMessage("Model file '" + filename + " does not contain a meta.xml file. aborting");
        return;
    }

    QFile metaFile(metaFileName);
    if (!metaFile.open(QFile::ReadOnly | QFile::Text)) {
        addStatusMessage("could not open meta.xml!");
        return;
    }

    QDomDocument doc;
    if (!doc.setContent(&metaFile)) {
        addStatusMessage( "could not parse meta.xml!");
        return;
    }
    QDomElement root = doc.documentElement();
    if (root.tagName() != "model") {
        addStatusMessage("meta.xml file in model '" + filename + "' is malformed.");
    }
    QDomElement typeEl = root.firstChildElement("type");
    if (typeEl.isNull()) {
        addStatusMessage("meta.xml file in model '" + filename + "' is malformed.");
    }
    if (typeEl.text() != "kinect_pcl") {
        QMessageBox::warning(this, "Error importing model", "The model file '" + filename + "' contains a model of type '" + typeEl.text() + "', whereas we need a model of type 'kinect_pcl'.");
        return;
    }

    QDomElement nameEl = root.firstChildElement("name");
    if (nameEl.isNull()) {
        addStatusMessage("meta.xml file in model '" + filename + "' is malformed.");
    }
    QString class_model_name = nameEl.text();

    if (class_model_name.count(".") != 1) {
        addStatusMessage("Cannot split '" + class_model_name + "' into class & model name. Please do so manually.");
    } else {
        QString modelname, classname;
        QStringList split = class_model_name.split('.');
        classname = split.at(0);
        modelname = split.at(1);
        ui->objectClassEdit->setText(classname);
        ui->objectNameEdit->setText(modelname);
    }

    QDomElement facesEl = root.firstChildElement("faces");
    QString facesTxt = facesEl.text();
    bool conv_ok = false;
    int numFaces = facesTxt.toInt(&conv_ok);
    if (!conv_ok) {
        QMessageBox::warning(this, "Error importing model", "Could not read number of faces from '" + filename + "'.");
        return;
    }

    QStringList facesFileNames;
    Q_FOREACH(const QString& ef, extracted_files) {
        if (ef.contains("dense_face_") && ef.endsWith(".pcd")) {
            facesFileNames.append(ef);
        }
    }
    if (facesFileNames.size() != numFaces) {
        QMessageBox::warning(this, "Error importing model", "Mismatch between number of faces in meta.xml and actual number of faces in model.");
        numFaces = facesFileNames.size();
    }

    Q_FOREACH(const QString& fn, facesFileNames) {
        comthread->loadPointCloud(fn);
    }
}


void MainWindow::on_actionDelete_triggered()
{
    QStringList clouds;
    QList<QListWidgetItem*> items = ui->cloudListWidget->selectedItems();
    Q_FOREACH(QListWidgetItem* item, items)
        clouds.append(item->text());
    comthread->removeCloud(clouds);
    Q_FOREACH(QListWidgetItem* item, items) {
        delete ui->cloudListWidget->takeItem(ui->cloudListWidget->row(item));
    }
}
