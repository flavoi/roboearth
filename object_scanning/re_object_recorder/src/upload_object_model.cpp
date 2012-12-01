/** \file upload_object_model.cpp
 * \brief command line tool for uploading object models to the roboearth DB
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
 * \author Daniel Di Marco
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

#include <iostream>
#include <string>
#include <boost/program_options.hpp>

#include <QString>
#include <QStringList>
#include <QDateTime>
#include <QFile>
#include <QDebug>
#include <QDomDocument>

#include <ros/ros.h>
#include <re_srvs/SetObject.h>
#include <re_srvs/GetObject.h>
#include <re_srvs/UpdateObjectBinaryFile.h>
#include <re_object_recorder/qminizip.h>
#include "owldescriptioncreator.h"
#include "uploadhelpers.h"

namespace po = boost::program_options;

using std::cout;
using std::cerr;
using std::endl;
using std::string;

// TODO: more operations?

class UploadObjectModelNode {
public:
    UploadObjectModelNode(int argc, char *argv[]) : desc("Usage"), update(false), no_owl(false) {
        desc.add_options()
                ("help", "show this help message")
                ("key", po:: value<string>(), "RoboEarth API key (get yours from http://api.roboearth.org)")
                ("name", po::value<string>(),"name of the object")
                ("class", po::value<string>(), "object class of the model")
                ("description", po::value<string>(), "human-readable description of the model")
                ("type", po::value<string>(), "model type (default value: 'ColoredPointCloudModel'")
                ("update", "signals that the given file should be updated/added to the object in the database")
                ("no-owl", "signals that no owl file should be generated (might cause upload to fail)")
                ("force", "upload object model regardless of model name/class inconsistencies")
                ("file", po::value<string>(), "zipped model file")
                ;

        po::positional_options_description pos_opt_descs;
        pos_opt_descs.add("file", 1);
        po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().positional(pos_opt_descs).run();
        po::store(parsed, vm);
        po::notify(vm);
    }

    bool checkArguments() {
        if (vm.count("help")) {
            printUsage();
            return false;
        }

        if (!vm.count("file")) {
            cerr << "no model file given." << endl;
            printUsage();
            return false;
        }
        file = vm["file"].as<string>();
        if (!vm.count("key")) {
            cerr << "API key not given. Please specify an RoboEarth API key with --key=<KEY>'" << endl;
            printUsage();
            return false;
        }

        if ((file.length() > 4) && (file.substr(file.length() - 4) == ".zip")) {
            // TODO: write model name/class from command line in zip file
            QMiniZip mz(QString::fromStdString(file), QMiniZip::UNZIP);

            QByteArray ba = mz.getFile("meta.xml");
            if (ba.isEmpty()) {
                cerr << "WARNING: could not read meta.xml file in model!" << endl;
            } else {
                QDomDocument dom("meta");
                dom.setContent(ba);

                QDomElement modelEl = dom.firstChildElement("model");
                QDomElement nameEl = modelEl.firstChildElement("name");
                QString nameTxt = nameEl.text();

                if (nameTxt.count(".") == 1) {
                    clss = nameTxt.split('.')[0].toStdString();
                    name = nameTxt.split('.')[1].toStdString();
                } else {
                    cerr << "WARNING: could not parse name in model: '" << nameTxt.toStdString() << "'" << endl;
                }
            }
        }
        if (name.empty()) {
            if (!vm.count("name")) {
                cerr << "ERROR: model name neither in meta.xml nor on command line found" << endl;
                return false;
            }
            name = vm["name"].as<string>();
        }
        if (clss.empty()) {
            if (!vm.count("class")) {
                cerr << "ERROR: model class neither in meta.xml nor on command line found" << endl;
                return false;
            }
            clss = vm["class"].as<string>();
        }
        if (vm.count("name") && (vm["name"].as<string>() != name)) {
            cerr << "WARNING: model name from command line does not match with model name from model's meta.xml: " << endl
                 << "\tname from command line: " << vm["name"].as<string>() << endl
                 << "\tname from meta.xml....: " << name << endl;
            if (!vm.count("force")) {
                return false;
            } else {
                cerr << "using model name from command line" << endl;
                name = vm["name"].as<string>();
            }
        }
        if (vm.count("class") && (vm["class"].as<string>() != clss)){
            cerr << "WARNING: model class from command line does not match with model class from model's meta.xml: " << endl
                 << "\tclass from command line: " << vm["class"].as<string>() << endl
                 << "\tclass from meta.xml....: " << clss << endl;
            if (!vm.count("force")) {
                return false;
            } else {
                cerr << "using model class from command line" << endl;
                clss = vm["class"].as<string>();
            }
        }

        key = vm["key"].as<string>();
        type = !vm.count("type") ? "ColoredPointCloudModel" : vm["type"].as<string>();
        description = !vm.count("description") ? "" : vm["description"].as<string>();

        update = vm.count("update");
        no_owl = vm.count("no-owl");

        return true;
    }

    bool upload() {
        // read file into FileMsg
        std::vector<re_msgs::File> fileMsgs;
        QString filename = QString::fromStdString(file);
        QString actualFilename = filename.split('/', QString::SkipEmptyParts).last();
        QFile f(filename);

        if (!f.open(QIODevice::ReadOnly)) {
            cerr << "Could not read file " << filename.toStdString() << endl;
            return false;
        }

        QByteArray data = f.readAll();
        re_msgs::File fileMsg;
        fileMsg.name = actualFilename.toStdString();
        fileMsg.data.resize(data.size());
        std::copy(data.data(), data.data()+data.size(), fileMsg.data.begin());
        fileMsgs.push_back(fileMsg);

        // upload
        UploadBase *uploadGeneric = NULL;
        if (update) {
            UpdateBinary *uploadSpecific = new UpdateBinary();
            uploadSpecific->fillRequest(key, clss + "." + name, fileMsg);

            uploadGeneric = uploadSpecific;
        } else {
            UploadSetNewObject *uploadSpecific = new UploadSetNewObject();
            OWLDescriptionCreator owldc(QString::fromStdString(type), QString::fromStdString(clss), QDateTime::currentDateTime());
            uploadSpecific->fillRequest(key, name, clss,
                                no_owl ? "" : owldc.getOwlDescription().toStdString(),
                                description, fileMsgs);

            uploadGeneric = uploadSpecific;
        }
        if (!uploadGeneric->waitForService())
            return false;
        if (!uploadGeneric->check())
            return false;

        return uploadGeneric->upload();
    }

private:
    void printUsage() const {
        cout << desc << "\n";
        cout << "For example: " << endl
             << " upload a new model:\tupload_object_model --key <API-KEY> kinectmodel.zip" << endl
             << " update a binary file:\tupload_object_model --key <API-KEY> --update kinectmodel.zip " << endl << endl
             << "Please note that re_comm must be running for this program to work." << endl;
    }

    ros::NodeHandle nh;
    po::options_description desc;
    po::variables_map vm;
    string key, name, clss, type, description, file;
    bool update, no_owl;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "upload_object_model");

    UploadObjectModelNode um(argc, argv);
    if (!um.checkArguments())
        return 1;
    if (um.upload()) {
        cout << "upload completed successfully" << endl;
    } else {
        cerr << "upload failed" << endl;
        return false;
    }

    return 0;
}
