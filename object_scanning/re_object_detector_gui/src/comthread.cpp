/** \file comthread.cpp
 * \brief Thread for ROS communication
 *
 * Thread class for communicating via ROS messages, i.e. for publishing model directories to object detection algorithms and receiving of camera images.
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

#include "comthread.h"

#include <ros/ros.h>
#include <re_vision/SearchFor.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <QDebug>

#include <re_vision/SearchFor.h>

ComThread::ComThread(QObject *parent) :
    QThread(parent), image_transport(nh)
{
    zaragoza_client = nh.serviceClient<re_vision::SearchFor>("/re_vision/search_for");
    zaragoza_model_dir_pub = nh.advertise<std_msgs::String>("/re_vslam/new_model", 1);

    kinect_model_dir_pub = nh.advertise<std_msgs::String>("/re_kinect/model_path", 1);

    image_sub = image_transport.subscribe("/camera/rgb/image_color", 1, &ComThread::imageCb, this);
    kinect_detected_objects_sub = nh.subscribe<re_kinect_object_detector::DetectionResult>("/re_kinect/detection_results", 10, boost::bind(&ComThread::kinect_detected_objectCb, this, _1));
}

void ComThread::run()
{
    ros::spin();
}

void ComThread::publishModelDir(QString model_dir, QString model_type, QString model_name) {
    std_msgs::String msg;
    msg.data = model_dir.toStdString();
    if ((model_type == "3D") && zaragoza_client.exists()) {
        zaragoza_model_dir_pub.publish(msg);
        if (std::find(zaragoza_model_names.begin(), zaragoza_model_names.end(),
                      model_name.toStdString()) == zaragoza_model_names.end()) {
            zaragoza_model_names.push_back(model_name.toStdString());
        }
    }
    else if (model_type == "kinect_pcl") {
        kinect_model_dir_pub.publish(msg);
    }

}

QImage cvMatToQImage(const cv::Mat& image)
{
    QImage qImage(
        image.data,
        image.size().width,
        image.size().height,
        image.step,
        QImage::Format_RGB888
    );

    return qImage.rgbSwapped();
}

void ComThread::kinect_detected_objectCb(const re_kinect_object_detector::DetectionResultConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg->Image,"bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

     for(size_t i=0; i<msg->Detections.size(); i++) {
         const re_msgs::DetectedObject&  detectedObject = msg->Detections.at(i);

         for (size_t j=0; j < detectedObject.points2d.size(); j++) {
             if (detectedObject.points2d.at(j).y < cv_ptr->image.size().height &&
                     detectedObject.points2d.at(j).y > 0 &&
                     detectedObject.points2d.at(j).x < cv_ptr->image.size().width &&
                     detectedObject.points2d.at(j).x > 0)
                 cv_ptr->image.at<cv::Vec3b>(detectedObject.points2d.at(j).y, detectedObject.points2d.at(j).x) = cv::Vec3b(0, 0, 255);
         }
     }
     emit updateKinectDetectionImg(cvMatToQImage(cv_ptr->image));
}

void ComThread::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    if (zaragoza_model_names.empty())
        return;

    re_vision::SearchForRequest req;
    req.Image = *msg;
    req.Objects = zaragoza_model_names;
    req.MaxPointsPerObject = -1;

    re_vision::SearchForResponse resp;
    zaragoza_client.call(req, resp);


//    qDebug() << "detections length: " << resp.Detections.size();
    if (resp.Detections.size() == 0)
        return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // mark object pixels
    bool marked_one = false;
    for(int i=0; i<resp.Detections.size(); i++) {
        const re_msgs::DetectedObject&  detectedObject = resp.Detections.at(i);

        for (int j=0; j < detectedObject.points2d.size(); j++) {
            marked_one = true;
            cv_ptr->image.at<cv::Vec3b>(detectedObject.points2d.at(j).y, detectedObject.points2d.at(j).x) = cv::Vec3b(0, 0, 255);
        }
    }

    if (marked_one)
        emit updateZaragozaDetectionImg(cvMatToQImage(cv_ptr->image));
}
