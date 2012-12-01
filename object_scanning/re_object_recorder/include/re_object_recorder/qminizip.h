/** \file qminizip.h
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

#ifndef QMINIZIP_H
#define QMINIZIP_H

#include <QString>
#include <QFileInfo>

/**
 * A simple utility class to create and extract ZIP files.
 * It is based on <a href="http://www.winimage.com/zLibDll/minizip.html">Minizip</a> by Gilles Vollant.
 **/
class QMiniZip {
public:

    enum OpenMode {ZIP_CREATE, ZIP_APPEND, UNZIP} ;

    QMiniZip(QString zipfile_, OpenMode mode_);
    ~QMiniZip();

    /**
     * Add a file to the zip archive.
     * @param fileInfo the file to add
     * @param nameInZip optional: the name of the file in the zip archive
     **/
    void addFile(const QFileInfo& fileInfo, const QString& nameInZip="");

    /**
     * Unzips the contents of the zip archive in the given path.
     * @param target_path
     * @return the list of filenames that was unzipped
     **/
    QStringList unzip(QDir target_path);
    /**
     * Gets the contents of the current zip file.
     **/
    QStringList getContents();

    QByteArray getFile(const QString &filename);
protected:
    QString do_extract_currentfile(const QDir& target_dir);

    QFileInfo zipFileInfo;

    OpenMode mode;
    void *zf;
    void *uf;


};

#endif // QMINIZIP_H
