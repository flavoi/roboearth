/** \file qminizip.cpp
 * \brief Utility class to create and extract ZIP files.
 *
 * This class uses Gilles Vollant's minizip.
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

#include "re_object_recorder/qminizip.h"

#include "minizip/zip.h"
#include "minizip/unzip.h"
#include <stdexcept>

#include <QDateTime>
#include <QDebug>
#include <QDir>

QMiniZip::QMiniZip(QString zipfile_, OpenMode mode_) : zipFileInfo(zipfile_), mode(mode_) {
    if (mode == ZIP_CREATE /*|| mode == ZIP_APPEND*/) {
//        qDebug() << "creating zip file" << (int)(mode == ZIP_APPEND);
        zf = zipOpen64(qPrintable(zipfile_), mode == ZIP_APPEND);
        if (zf == NULL)
            throw std::runtime_error("could not open zip file");
    }
    else if (mode == UNZIP) {
        uf = unzOpen64(qPrintable(zipfile_));
    }
    else
        throw std::logic_error("not implemented yet");
}

QMiniZip::~QMiniZip() {
    if (mode == ZIP_APPEND || mode == ZIP_CREATE)
        zipClose(zf, NULL);
    else if (mode == UNZIP)
        unzClose(uf);
    else
        throw std::logic_error("unknown mode in destructor");
}

void QMiniZip::addFile(const QFileInfo& fileInfo, const QString& nameInZip) {
    if (mode != ZIP_CREATE && mode != ZIP_APPEND) {
        throw std::logic_error("cannot add to zip file that was opened in UNZIP mode");
    }

    zip_fileinfo zi;
    zi.dosDate = 0;
    zi.internal_fa = 0;
    zi.external_fa = 0;

    QDateTime lastModified = fileInfo.lastModified();
    zi.tmz_date.tm_hour = lastModified.time().hour();
    zi.tmz_date.tm_min = lastModified.time().minute();
    zi.tmz_date.tm_sec = lastModified.time().second();
    zi.tmz_date.tm_mday = lastModified.date().day();
    zi.tmz_date.tm_mon = lastModified.date().month() - 1;
    zi.tmz_date.tm_year = lastModified.date().year();

    QString niz = nameInZip.isEmpty() ? fileInfo.fileName() : nameInZip;
    int opt_compress_level = Z_DEFAULT_COMPRESSION;
    int err = zipOpenNewFileInZip3_64(zf, qPrintable(niz), &zi,
                                      NULL,0,NULL,0,NULL,
                                      (opt_compress_level != 0) ? Z_DEFLATED : 0,
                                      opt_compress_level,0,
                                      -MAX_WBITS, DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY,
                                      NULL, 0, 1);
    if (err != ZIP_OK)
    {
        throw std::runtime_error("could not create file in zip");
    }

    const size_t WRITE_BUFFER_SIZE = 16384;
    char *buf = new char[WRITE_BUFFER_SIZE];
    FILE *fin = fopen64(qPrintable(fileInfo.absoluteFilePath()), "rb");
    int size_read;
    do {
        size_read = fread(buf, 1, WRITE_BUFFER_SIZE, fin);

        if (size_read > 0)
            err = zipWriteInFileInZip(zf, buf, size_read);
    } while ((err == ZIP_OK) && (size_read > 0));

    if (err != ZIP_OK) {
        throw std::runtime_error("error reading input file");
    }

    fclose(fin);
    err = zipCloseFileInZip(zf);
    if (err != ZIP_OK) {
        throw std::runtime_error("error closing newly created file in zip");
    }
    delete [] buf;
}

QStringList QMiniZip::unzip(QDir target_path) {
    QStringList result;
    unz_global_info64 gi;
    int err;

    unzGoToFirstFile(uf);

    err = unzGetGlobalInfo64(uf, &gi);
    if (err!=UNZ_OK) {
        qDebug() << "error with zipfile in unzGetGlobalInfo";
    }

//    qDebug() << "number of entries: " << gi.number_entry;

    for (size_t i=0; i<gi.number_entry; i++)
    {
        QString extracted_file = do_extract_currentfile(target_path);
        if (extracted_file.isEmpty())
            break;
        result.append(extracted_file);

        if ((i+1)<gi.number_entry)
        {
            err = unzGoToNextFile(uf);
            if (err!=UNZ_OK)
            {
                printf("error %d with zipfile in unzGoToNextFile\n",err);
                break;
            }
        }
    }
    return result;
}

QString QMiniZip::do_extract_currentfile(const QDir& target_dir) {
    char filename_inzip[256];
    unz_file_info64 file_info;
    int err;
    //        uLong ratio=0;
    err = unzGetCurrentFileInfo64(uf,&file_info,filename_inzip,sizeof(filename_inzip),NULL,0,NULL,0);
    if (err != UNZ_OK)
        throw std::runtime_error("error during unzGetCurrentFileInfo");

    const size_t WRITE_BUFFER_SIZE = 16384;
    char *buf = new char[WRITE_BUFFER_SIZE];

    QString filename_inzip_qstr(filename_inzip);
    QString path = filename_inzip_qstr.left(filename_inzip_qstr.lastIndexOf("/"));
    if (!path.isEmpty() && path != filename_inzip_qstr) {
        QFileInfo p(target_dir.filePath(path));
        if (!p.exists()) {
            //                qDebug() << "creating path " << path;
            target_dir.mkpath(path);
        }
    }

    err = unzOpenCurrentFilePassword(uf,NULL);
    if (err!=UNZ_OK)
    {
        printf("error %d with zipfile in unzOpenCurrentFilePassword\n",err);
    }

    QString resulting_filename(target_dir.filePath(filename_inzip_qstr));

    FILE *fout;
    fout=fopen64(qPrintable(resulting_filename),"wb");
    do
    {
        err = unzReadCurrentFile(uf, buf, WRITE_BUFFER_SIZE);
        if (err<0)
        {
            printf("error %d with zipfile in unzReadCurrentFile\n",err);
            break;
        }
        if (err>0)
            if (fwrite(buf,err,1,fout)!=1)
            {
                printf("error in writing extracted file\n");
                err=UNZ_ERRNO;
                break;
            }
    } while (err>0);
    if (fout)
        fclose(fout);

    if (err==UNZ_OK)
    {
        err = unzCloseCurrentFile (uf);
    }


    delete [] buf;
    return resulting_filename;
}

QByteArray QMiniZip::getFile(const QString& filename) {
    unz_file_info64 file_info;
    QByteArray result;
    char filename_inzip[256];

    unz_global_info64 gi;
    int err;

    unzGoToFirstFile(uf);

    err = unzGetGlobalInfo64(uf, &gi);
    if (err!=UNZ_OK) {
        qDebug() << "error with zipfile in unzGetGlobalInfo";
    }

    for (size_t i=0; i<gi.number_entry; i++)
    {
        err = unzGoToNextFile(uf);
        if (err != UNZ_OK) {
            return result;
        }
        err = unzGetCurrentFileInfo64(uf,&file_info,filename_inzip,sizeof(filename_inzip),NULL,0,NULL,0);
        err = unzGetCurrentFileInfo64(uf,&file_info,filename_inzip,sizeof(filename_inzip),NULL,0,NULL,0);
        if (err != UNZ_OK)
            return result;
        QString fn(filename_inzip);
        if (fn.endsWith(filename)) {
            break;
        }
    }

    const size_t WRITE_BUFFER_SIZE = 16384;
    static char buf[WRITE_BUFFER_SIZE];

    err = unzOpenCurrentFilePassword(uf, NULL);
    if (err!=UNZ_OK) {
        printf("error %d with zipfile in unzOpenCurrentFilePassword\n",err);
    }

    do {
        err = unzReadCurrentFile(uf, buf, WRITE_BUFFER_SIZE);
        if (err<0) {
            printf("error %d with zipfile in unzReadCurrentFile\n",err);
            break;
        }
        if (err>0) {
            result.append(buf, err);
        }
    } while (err>0);
    if (err==UNZ_OK) {
        err = unzCloseCurrentFile (uf);
    }

    return result;
}

QStringList QMiniZip::getContents() {
    QStringList result;
    unz_global_info64 gi;
    int err;

    unzGoToFirstFile(uf);

    err = unzGetGlobalInfo64(uf, &gi);
//    if (err!=UNZ_OK)
//        printf("error %d with zipfile in unzGetGlobalInfo \n", err);

    for (uLong i=0;i<gi.number_entry;i++) {
        char filename_inzip[256];
        unz_file_info64 file_info;
        err = unzGetCurrentFileInfo64(uf,&file_info,filename_inzip,sizeof(filename_inzip),NULL,0,NULL,0);
        if (err!=UNZ_OK)
        {
//            printf("error %d with zipfile in unzGetCurrentFileInfo\n",err);
            break;
        }
        result.append(QString::fromLocal8Bit(filename_inzip));

        if ((i+1)<gi.number_entry)
        {
            err = unzGoToNextFile(uf);
            if (err!=UNZ_OK)
            {
                printf("error %d with zipfile in unzGoToNextFile\n",err);
                break;
            }
        }
    }

    return result;
}
