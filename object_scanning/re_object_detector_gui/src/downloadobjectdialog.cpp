/** \file downloadobjectdialog.cpp
 * \brief Dialog for querying and downloading of object models
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


#include "downloadobjectdialog.h"
#include "ui_downloadobjectdialog.h"

#include <re_object_recorder/qminizip.h>

#include <ros/service.h>
#include <re_srvs/QueryObjects.h>
#include <re_srvs/SearchObjects.h>
#include <re_srvs/GetObjectBinaryFile.h>

#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QtCore>
#include <QProgressDialog>

const QString DOWNLOAD_SERVICE = "/re_comm/get_object_binary_file";
const QString QUERY_SERVICE = "/re_comm/search_objects";

DownloadObjectDialog::DownloadObjectDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DownloadObjectDialog)
{
    ui->setupUi(this);
    ui->downloadDirEdit->setText(QDir::tempPath());
}

DownloadObjectDialog::~DownloadObjectDialog()
{
    delete ui;
}

void DownloadObjectDialog::on_cancelButton_clicked() {
    emit reject();
}

void DownloadObjectDialog::addTableEntry(QString uid, QString filename, QString fileurl) {
    int row = ui->resultsTableWidget->rowCount();
    ui->resultsTableWidget->insertRow(row);
    QTableWidgetItem *checkItem = new QTableWidgetItem();
    checkItem->setCheckState(Qt::Unchecked);
    QTableWidgetItem *uidItem = new QTableWidgetItem(uid);
    QTableWidgetItem *filenameItem = new QTableWidgetItem(filename);
    QTableWidgetItem *fileURLItem = new QTableWidgetItem(fileurl);

    ui->resultsTableWidget->setItem(row, 0, checkItem);
    ui->resultsTableWidget->setItem(row, 1, uidItem);
    ui->resultsTableWidget->setItem(row, 2, filenameItem);
    ui->resultsTableWidget->setItem(row, 3, fileURLItem);
}

/// wrapper function for use with QFutureWatcher
bool searchObjectsServiceCall(re_srvs::SearchObjectsRequest& req, re_srvs::SearchObjectsResponse* resp) {
    return ros::service::call(QUERY_SERVICE.toStdString(), req, *resp);
}

void DownloadObjectDialog::on_searchButton_clicked() {
    ui->resultsTableWidget->clearContents();
    ui->resultsTableWidget->setRowCount(0);

    QString searchQStr = ui->searchEdit->text().trimmed();

    // FIXME: re_comm seems to crash when a space (' ') is sent
    if (searchQStr.isEmpty()) {
        ui->queryFeedbackLbl->setText("Empty queries are not allowed");
        return;
    }

    if (!ros::service::exists(QUERY_SERVICE.toStdString(), true)) {
        QMessageBox::warning(this, "Error", "Could not reach syntactic query service. Is re_comm running?");
        return;
    }

    re_srvs::SearchObjectsRequest req;
    req.searchID = searchQStr.toStdString();
    re_srvs::SearchObjectsResponse resp;
    ui->queryFeedbackLbl->setText("Querying...");

    {
        QProgressDialog dlg("Querying RoboEarth...", "Cancel", 0, 0, this);
        dlg.setCancelButton(NULL);
        QFutureWatcher<bool> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), &dlg, SLOT(reset()));
        QObject::connect(&dlg, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        // NOTE: QtConcurrent::run creates copies of its arguments before calling the function
        QFuture<bool> future = QtConcurrent::run(&searchObjectsServiceCall, req, &resp);
        futureWatcher.setFuture(future);

        dlg.exec();

        futureWatcher.waitForFinished();
        if (dlg.wasCanceled()) {
            ui->queryFeedbackLbl->setText("Query cancelled by user");
            return;
        } else if ((future.result() == true) && resp.success) {
            ui->queryFeedbackLbl->setText("Query successful");
        } else {
            ui->queryFeedbackLbl->setText("Query failed");
            return;
        }
    }

    for(unsigned int i=0; i<resp.objects.size(); i++) {
        QString uid = QString::fromStdString(resp.uids.at(i));
        re_msgs::StringArray filenames = resp.filenames.at(i);
        re_msgs::StringArray fileURLS = resp.fileURLs.at(i);

        for (unsigned int j=0; j<filenames.list.size(); j++) {
            addTableEntry(uid, QString::fromStdString(filenames.list.at(j)), QString::fromStdString(fileURLS.list.at(j)));
        }
    }
}

void DownloadObjectDialog::on_chooseDownloadDirButton_clicked() {
    QString dir = QFileDialog::getExistingDirectory(this, "Choose a target directory", ui->downloadDirEdit->text());
    if (dir.isEmpty())
        return;

    ui->downloadDirEdit->setText(dir);
}

/// wrapper function for use with QFutureWatcher
bool downloadObjectsServiceCall(re_srvs::GetObjectBinaryFileRequest& req, re_srvs::GetObjectBinaryFileResponse *resp) {
    return ros::service::call(DOWNLOAD_SERVICE.toStdString(), req, *resp);
}

bool DownloadObjectDialog::downloadFile(QString uid, QString filename, QDir target_path) {
    re_srvs::GetObjectBinaryFileRequest req;
    req.objectUID = uid.toStdString();
    req.filename = filename.toStdString();
    re_srvs::GetObjectBinaryFileResponse resp;
    {
        QProgressDialog dlg("Downloading Objects", "Cancel", 0, 0, this);
        dlg.setCancelButton(NULL);
        QFutureWatcher<bool> futureWatcher;
        QObject::connect(&futureWatcher, SIGNAL(finished()), &dlg, SLOT(reset()));
        QObject::connect(&dlg, SIGNAL(canceled()), &futureWatcher, SLOT(cancel()));
        // NOTE: QtConcurrent::run creates copies of its arguments before calling the function
        QFuture<bool> future = QtConcurrent::run(&downloadObjectsServiceCall, req, &resp);
        futureWatcher.setFuture(future);

        dlg.exec();

        futureWatcher.waitForFinished();
        if (dlg.wasCanceled()) {
            QMessageBox::warning(this, "Download cancelled", "Download cancelled by user.");
            return false;
        } else if ((future.result() == true) && resp.success) {
            // download successful
        } else {
            QMessageBox::warning(this, "Download failed", "Downloading of '" + uid + "' failed!");
            return false;
        }
    }

    QFile out_file(target_path.filePath(QString::fromStdString(resp.file.name)));
    if (!out_file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "Download failed", "Could not open '" + out_file.fileName() + "' for writing.");
        return false;
    }
    qDebug() << "writing to " << out_file.fileName();

    // TOOD: what if memory full?
    char *buffer = new char[resp.file.data.size()];
    std::copy(resp.file.data.begin(), resp.file.data.end(), buffer);
    qint64 written = out_file.write(buffer, resp.file.data.size());
    delete [] buffer;

    if (written < resp.file.data.size()) {
        QMessageBox::warning(this, "Download failed", "Wrote only " + QString::number(written) + " of "
                             + QString::number(resp.file.data.size()) + " bytes to " + out_file.fileName() + "!");
        return false;
    }

    return true;
}

void DownloadObjectDialog::on_downloadButton_clicked() {
    bool service_checked = false;
    QDir target_path(ui->downloadDirEdit->text());
    for(int i=0; i<ui->resultsTableWidget->rowCount(); i++) {
        QTableWidgetItem *checkboxItem = ui->resultsTableWidget->item(i, 0);
        QTableWidgetItem *uidItem = ui->resultsTableWidget->item(i, 1);
        QTableWidgetItem *fileURLItem = ui->resultsTableWidget->item(i, 3);
        if (checkboxItem->checkState() == Qt::Checked) {
            // get actual file name (i.e. last part of URL)
            QStringList fileURLSplitted = fileURLItem->text().split('/', QString::SkipEmptyParts);
            QString realFilename = fileURLSplitted.last();

            if (!service_checked && !ros::service::exists(DOWNLOAD_SERVICE.toStdString(), true)) {
                QMessageBox::warning(this, "Error", "Could not reach download service. Is re_comm running?");
                return;
            }
            service_checked = true;

            bool downloaded = downloadFile(uidItem->text(), realFilename, target_path);
            if (downloaded && realFilename.endsWith(".zip", Qt::CaseInsensitive)) {
                qDebug() << "extracting into " << target_path;
                // unpack
                QMiniZip mz(target_path.filePath(realFilename), QMiniZip::UNZIP);
                mz.unzip(target_path);

                QStringList zipContents = mz.getContents();
                qDebug() << zipContents;
                QString created_dir_name = zipContents.first().split('/', QString::SkipEmptyParts).first();

                this->downloaded_models.append(target_path.filePath(created_dir_name));
            }
        }
    }

    emit accept();
}
