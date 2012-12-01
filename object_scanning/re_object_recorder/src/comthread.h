/** \file comthread.h
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

#ifndef COMTHREAD_H
#define COMTHREAD_H

#include <QThread>
#include <QStringList>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/subscriber.h>
#include <QMutex>
#include <QDir>
#include <tf/transform_listener.h>

namespace pcl{
	namespace visualization {
class CloudViewer;
};
};

typedef pcl::PointXYZRGB PointType;

/**
 * Main ROS communication thread. Also receives and stores point clouds.
 **/
class ComThread : public QThread
{
    Q_OBJECT
public:
    explicit ComThread(QObject *parent = 0);

    virtual ~ComThread();

    /**
     * Point cloud callback
     * @param pcl_msg the received point cloud message
     **/
    void pointCloudReceived(const sensor_msgs::PointCloud2ConstPtr& pcl_msg);

    /**
     * Load and add a pointcloud from a .pcd file.
     * @param filename path to the pcd file
     **/
    void loadPointCloud(const QString& filename);

    /**
     * Create a preview image of the object. It gets saved in model_dir.
     * @param model_dir target directory for the image file
     * @return true when a preview image was created
     **/
    bool createPreview(QDir& model_dir);

Q_SIGNALS:
    /**
     * Add a status message to the message log QTextEdit in the UI.
     * @param QString message to show
     **/
    void addStatusMessage(QString);
    /**
     * Signals that a new point cloud was received
     * @param QString the name of the cloud in the viewer or "-" if the cloud was not recorded
     **/
    void newCloudReceived(QString);

public Q_SLOTS:
    void recording(bool startstop);
    /**
     * Resets the cloud viewer and removes all recorded point clouds.
     **/
    void reset();
    /**
     * Removes the given point clouds.
     * @param QStringList the names of the point clouds to remove
     **/
    void removeCloud(QStringList);
    /**
     * Selects the given point clouds.
     * @param the names of the point clouds to select
     **/
    void selectCloud(QStringList);

    /**
     * Updates the new bounding box size in the cloud viewer.
     * @param double new bounding box size
     **/
    void changedBoxSize(double);

    /**
     * Export an object recognition model compatible with re_vision.
     * @param parent_dir directory in which to store the model
     * @param model_name name of the model
     **/
    void createRe_visionModel(QDir& parent_dir, const QString& model_name);
    /**
     * Export a kinect object model (i.e. a set of dense .pcd files)
     * @param parent_dir directory in which to store the model
     * @param model_name name of the model
     **/
    void createKinectModel(QDir& parent_dir, const QString& model_name);

    /**
     * The user changed the number of maximum scans in the UI.
     * @param int new maximum number of scans
     **/
    void maxScanNumberChanged(int);

protected:
    void run();

    /**
     * Merge all recorded point clouds.
     * @param merged_pcl output parameter for the merged point cloud
     **/
    void getMergedPointCloud(pcl::PointCloud<PointType>& merged_pcl);
    /**
     * Merge all point clouds and save them in a pcd file.
     * @param QString filename
     **/
    void saveMergedPCL(QString);
    /**
     * Store all point clouds in a directory.
     * @param QDir target directory
     **/
    void saveFaces(QDir);
    /**
     * Add a pointcloud to the storage and to the visualization.
     * @return the id of the added point cloud
     * @param cloud the cloud to add
     **/
    int addPointCloud(boost::shared_ptr<pcl::PointCloud<PointType> > cloud);

    tf::TransformListener tf;
    ros::Subscriber sub;
    bool isRecording;
    std::map<int, boost::shared_ptr< pcl::PointCloud<PointType> > > pcls;
    int npoints;
    pcl::visualization::CloudViewer* cloudViewer;
    QMutex mutex;

    /// maximum number of pointclouds to keep in memory
    unsigned int max_pointclouds;
};

#endif // COMTHREAD_H

