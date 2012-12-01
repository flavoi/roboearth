/** \file comthread.cpp
 * \brief Thread for ROS communication
 *
 * Thread class for communicating via ROS messages, i.e. for receiving colored point clouds.
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

#include "comthread.h"
#include <ros/ros.h>
#include <QDebug>
#include <QDir>
#include <QPoint>
#include <QImage>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include "re_visionmodelcreator.h"

ComThread::ComThread(QObject *parent) :
        QThread(parent)
{
    ros::NodeHandle nh;
    sub = nh.subscribe<sensor_msgs::PointCloud2>("resulting_pcl",1,boost::bind(&ComThread::pointCloudReceived,this,_1));
    isRecording = false;
    npoints = 0;
    max_pointclouds = 100;

    cloudViewer = new pcl::visualization::CloudViewer("object point cloud");
}

ComThread::~ComThread() {
    delete cloudViewer;
}

void ComThread::run() {
    ros::Rate r(2);
    while (ros::ok() && !cloudViewer->wasStopped()) {
        ros::spinOnce();
        r.sleep();
    }
}

void ComThread::recording(bool startstop) {
    isRecording = startstop;
}

void ComThread::reset() {
    mutex.lock();
    npoints = 0;
    pcl::PointCloud<PointType>::Ptr emptycloud( new pcl::PointCloud<PointType>());

    for (std::map<int,boost::shared_ptr<pcl::PointCloud<PointType> > >::const_iterator it = pcls.begin(); it != pcls.end(); ++it) {
        cloudViewer->showCloud(emptycloud,boost::lexical_cast<std::string>(it->first));
    }

    pcls.clear();

    cloudViewer->showCloud(emptycloud);

    mutex.unlock();
    Q_EMIT newCloudReceived("-");
}

void ComThread::pointCloudReceived(const sensor_msgs::PointCloud2ConstPtr &pcl_msg) {
    ROS_INFO("Received PointCloud");

    boost::shared_ptr<pcl::PointCloud<PointType> > cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pcl_msg,*cloud);

    // get camera pose in respect to object
    geometry_msgs::PoseStamped origin, markerToRGBCamera;
    origin.header.frame_id = pcl_msg->header.frame_id;
    origin.header.stamp = pcl_msg->header.stamp;
    origin.pose.orientation.w = 1.f;
    try {
        tf.transformPose("/camera_rgb_optical_frame", origin, markerToRGBCamera);

        cloud->sensor_origin_ = Eigen::Vector4f(markerToRGBCamera.pose.position.x,
                                                markerToRGBCamera.pose.position.y,
                                                markerToRGBCamera.pose.position.z,
                                                0);
        cloud->sensor_orientation_ = Eigen::Quaternionf(markerToRGBCamera.pose.orientation.w,
                                                        markerToRGBCamera.pose.orientation.x,
                                                        markerToRGBCamera.pose.orientation.y,
                                                        markerToRGBCamera.pose.orientation.z);
    } catch(const std::exception& e) {
        ROS_WARN("could not get camera pose: %s", e.what());
        return;
    }

    if (!isRecording) {
        // just make the cloud yellow and display it
        BOOST_FOREACH(PointType& pt, cloud->points) {
            pt.rgb = 25590;
        }
        cloudViewer->showCloud(cloud,"current");
    } else {
        addPointCloud(cloud);
    }
}

/**
 * Load a pointcloud from a pcd file.
 * Checks whether the .pcd has padding after the header, and removes it it if needed.
 * (binary PCD files in versions < 1.0 required padding, PCL >= version 1.0 does not)
 * @param filename pcd file to load
 * @param cloud pointer to which the loaded cloud is written
 * @return 0 for success, != 0 otherwise
 **/
int read_binary_PCL(const std::string& filename,  pcl::PointCloud<PointType>::Ptr cloud) {
    std::ifstream in_file(filename.c_str());
    std::string token("DATA binary\n");

    const size_t BUFFER_SIZE = 4096;
    char buffer[BUFFER_SIZE];
    in_file.read(buffer, BUFFER_SIZE);
    if (in_file.gcount() < BUFFER_SIZE) {
        std::cerr << "could not read whole file" << std::endl;
        return -1;
    }

    bool found_binary = false;
    bool null_after_header = false;
    size_t matched_chars = 0;
    size_t header_length = 0;
    for(size_t i=0; i<BUFFER_SIZE; i++) {
        if (buffer[i] == token[matched_chars]) {
            matched_chars++;
        } else
            matched_chars = 0;

        if (matched_chars == token.length()) {
            found_binary = true;

            header_length = i;
            size_t num_nulls = 0;
            for(size_t j=header_length+1; j<BUFFER_SIZE; j++) {
                if (buffer[j] == '\0')
                    num_nulls++;
                else
                    break;
            }
            null_after_header = num_nulls > 10;
            if (null_after_header) {
                std::cout << "null bytes found after header, converting from old PCD binary format" << std::endl;
            }
            break;
        }
    }

    std::string tmp_filename;
    int temp_file_fd;
    if (null_after_header) {
        char tmp_name[] = "/tmp/pcd_file_XXXXXX";

        temp_file_fd = mkstemp(tmp_name);
        if (temp_file_fd < 0) {
            std::cerr << "could not create temporary file" << std::endl;
            return -1;
        }
        tmp_filename = tmp_name;

        std::ofstream tmp_file(tmp_filename.c_str());
        tmp_file.write(buffer, header_length+1);

        in_file.seekg(4096, std::ios::beg);
        while(in_file.good()) {
            in_file.read(buffer, BUFFER_SIZE);
            tmp_file.write(buffer, in_file.gcount());
        }
        in_file.close();
        tmp_file.close();
        pcl::io::loadPCDFile(tmp_filename, *cloud);
        remove(tmp_filename.c_str());
    } else {
        in_file.close();
        // just load it
        pcl::io::loadPCDFile(filename, *cloud);
    }

    return 0;
}

void ComThread::loadPointCloud(const QString &filename) {
    boost::shared_ptr<pcl::PointCloud<PointType> > cloud(new pcl::PointCloud<PointType>);
    // TODO: sensor origin/orientation? base frame?
    read_binary_PCL(filename.toStdString(), cloud);
    // TODO: move to parameter
    cloud->header.frame_id = "/PATTERN_BASE";

    int pcl_id = addPointCloud(cloud);
    Q_EMIT addStatusMessage("Loaded '" + filename + "' as scan number " + QString::number(pcl_id) + ".");
}

int ComThread::addPointCloud(boost::shared_ptr<pcl::PointCloud<PointType> > cloud) {
    QMutexLocker lock(&mutex);

    if (pcls.size() >= max_pointclouds) {
        Q_EMIT addStatusMessage(QString("Received a new point cloud, but we have already %1 pointclouds! Ignoring this one.").arg(pcls.size()));
        return -1;
    }

    npoints += cloud->width * cloud->height;

    int count = 0;
    while (pcls.find(count) != pcls.end()) {
        count ++;
    }
    pcls.insert(std::make_pair(count, cloud));

    Q_EMIT newCloudReceived(QString::number(count));
    cloudViewer->showCloud(cloud,boost::lexical_cast<std::string>(count));
    Q_EMIT addStatusMessage("Accumulated " + QString::number(npoints) + " points in " + QString::number(pcls.size()) + " point clouds");
    return count;
}

void ComThread::removeCloud(QStringList ids) {
    QMutexLocker lock(&mutex);

    Q_FOREACH(QString id, ids) {
        Q_EMIT addStatusMessage("Removing cloud " + id);
        if(pcls.find(id.toInt())!=pcls.end()) {
            pcls[id.toInt()]->points.clear();
            cloudViewer->showCloud(pcls[id.toInt()],id.toStdString());
            pcls.erase(pcls.find(id.toInt()));
        }
    }
}

void ComThread::selectCloud(QStringList newSelections) {
    QMutexLocker lock(&mutex);

    static QStringList alreadySelected;

    QStringList notselectedanymore;
    QStringList alreadyselectedandstillis;

    Q_FOREACH(QString id, alreadySelected) {
        if (newSelections.contains(id)) {
            alreadyselectedandstillis.append(id);
            newSelections.removeAll(id);
        } else {
            notselectedanymore.append(id);
        }
    }

    Q_FOREACH(QString id, notselectedanymore) {

        if(pcls.find(id.toInt())!=pcls.end()){
            cloudViewer->showCloud(pcls[id.toInt()],id.toStdString()); //Mauro, maybe a copy was necessary.
        } else {
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
            cloudViewer->showCloud(cloud,id.toStdString());
        }
    }

    Q_FOREACH(QString id, newSelections) {
        pcl::PointCloud<PointType>::Ptr copy(new pcl::PointCloud<PointType>());
        if(pcls.find(id.toInt())!=pcls.end()){
            copy = pcls[id.toInt()];
            BOOST_FOREACH(PointType& pt, copy->points) {
                pt.rgb = 255;
            }
        }
        cloudViewer->showCloud(copy, id.toStdString());
    }

    alreadySelected = newSelections;
    alreadySelected.append(alreadyselectedandstillis);
}

void ComThread::getMergedPointCloud(pcl::PointCloud<PointType> &merged_pcl) {
    merged_pcl.points.clear();
    for (std::map<int,boost::shared_ptr<pcl::PointCloud<PointType> > >::const_iterator it = pcls.begin(); it != pcls.end(); ++it) {
        pcl::PointCloud<PointType> cleaned_cloud;

        pcl::PassThrough<PointType> nan_remover;
        nan_remover.setInputCloud(it->second);
        nan_remover.setFilterFieldName("z");
        nan_remover.setFilterLimits(0.0, 10.0);
        nan_remover.filter(cleaned_cloud);

        if (!merged_pcl.points.size())
            merged_pcl = cleaned_cloud;
        else
            merged_pcl += cleaned_cloud;
    }
}

void ComThread::saveMergedPCL(QString filename) {
    pcl::PointCloud<PointType> mergedpcls;
    getMergedPointCloud(mergedpcls);

    pcl::io::savePCDFileBinary(filename.toStdString(), mergedpcls);
}

void ComThread::saveFaces(QDir model_dir) {
    int i=0;
    for (std::map<int,boost::shared_ptr<pcl::PointCloud<PointType> > >::const_iterator it = pcls.begin(); it != pcls.end(); ++it, ++i) {
        QString recording = QString("dense_face_%1.pcd").arg(i, 3, 10, QChar('0'));
        QString recording_file = model_dir.absoluteFilePath(recording);

        pcl::io::savePCDFileBinary(recording_file.toStdString(), *it->second);
    }
}

void ComThread::changedBoxSize(double size) {
    pcl::PointXYZ minp(-size,-size,.01);
    pcl::PointXYZ maxp(size,size,2*size+.01);

    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingboxcloud(new pcl::PointCloud<pcl::PointXYZ>());
    boundingboxcloud->push_back(pcl::PointXYZ(minp.x,minp.y,minp.z));
    boundingboxcloud->push_back(pcl::PointXYZ(minp.x,maxp.y,minp.z));
    boundingboxcloud->push_back(pcl::PointXYZ(minp.x,minp.y,maxp.z));
    boundingboxcloud->push_back(pcl::PointXYZ(minp.x,maxp.y,maxp.z));

    boundingboxcloud->push_back(pcl::PointXYZ(maxp.x,minp.y,minp.z));
    boundingboxcloud->push_back(pcl::PointXYZ(maxp.x,maxp.y,minp.z));
    boundingboxcloud->push_back(pcl::PointXYZ(maxp.x,minp.y,maxp.z));
    boundingboxcloud->push_back(pcl::PointXYZ(maxp.x,maxp.y,maxp.z));
    cloudViewer->showCloud(boundingboxcloud, "boundingbox");
}

void ComThread::createRe_visionModel(QDir &parent_dir, const QString& model_name) {
    QMutexLocker lock(&mutex);

    std::vector<boost::shared_ptr<pcl::PointCloud<PointType> > >  vec;
    for (std::map<int,boost::shared_ptr<pcl::PointCloud<PointType> > >::const_iterator it = pcls.begin(); it != pcls.end(); ++it) {
        vec.push_back(it->second);
    }

    pcl::PointCloud<PointType> merged_pcl;
    getMergedPointCloud(merged_pcl);

    re_visionModelCreator zmg(vec);
    zmg.createModel(parent_dir, merged_pcl, model_name);
}

void ComThread::createKinectModel(QDir &parent_dir, const QString &model_name) {
    QMutexLocker lock(&mutex);

    QString merged_file = parent_dir.absoluteFilePath("merged.pcd");
    qDebug() << "saving merged.pcd";
    saveMergedPCL(merged_file);
    qDebug() << "saving faces";
    saveFaces(parent_dir);

    qDebug() << "creating meta file";
    re_visionModelCreator::createMetaFile(parent_dir, model_name, "kinect_pcl", pcls.size(), 1);
}

void ComThread::maxScanNumberChanged(int value) {
    max_pointclouds = value;
}

bool ComThread::createPreview(QDir& model_dir) {
    QMutexLocker lock(&mutex);

    size_t max_area = 0;
    boost::shared_ptr<pcl::PointCloud<PointType> > best_face;
    QPoint best_minp, best_maxp;

    // iterate through all faces and find biggest possible preview
    for (std::map<int,boost::shared_ptr<pcl::PointCloud<PointType> > >::const_iterator it = pcls.begin(); it != pcls.end(); ++it) {
        const pcl::PointCloud<PointType>& pcl = *it->second;
        size_t i=0;
        QPoint minp, maxp;
        minp.setX(pcl.width);
        minp.setY(pcl.height);
        for (size_t u = 0; u < pcl.height; ++u)
        {
            for (size_t v = 0; v < pcl.width; ++v, ++i)
            {
                // If the point is invalid (i.e. belongs to background)
                if (!(isnan (pcl.points[i].x) || isnan (pcl.points[i].y) || isnan (pcl.points[i].z))) {
                    if (v < minp.x())
                        minp.setX(v);
                    if (v > maxp.x())
                        maxp.setX(v);
                    if (u < minp.y())
                        minp.setY(u);
                    if (u > maxp.y())
                        maxp.setY(u);
                }
            }
        }

        size_t area = (maxp.x() - minp.x() +1) * (maxp.y() - minp.y()+1);
        if (area > max_area) {
            max_area = area;
            best_face = it->second;
            best_minp = minp;
            best_maxp = maxp;
        }
    }

    if (max_area < 100)
        return false;

    const pcl::PointCloud<PointType>& pcl = *best_face;
    QImage img_color(best_maxp.x() - best_minp.x() +1, best_maxp.y() - best_minp.y()+1, QImage::Format_RGB32);
    size_t i = 0;

    for (size_t u = 0; u < pcl.height; ++u)
    {
        for (size_t v = 0; v < pcl.width; ++v, ++i)
        {
            if ((u >= best_minp.y()) && (u <= best_maxp.y()) &&
                    (v >= best_minp.x()) && (v <= best_maxp.x())) {
                int32_t rgb = *(int32_t*)(&pcl.points[i].rgb);
                char b = rgb & 0xFF;
                char g = (rgb & (0xFF << 8)) >> 8;
                char r = (rgb & (0xFF << 16)) >> 16;

                img_color.setPixel(v - best_minp.x(), u - best_minp.y(), qRgb(r, g, b));
            }
        }
    }

    return img_color.save(model_dir.filePath("preview.png"));
}
