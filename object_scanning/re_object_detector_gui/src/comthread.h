/** \file comthread.h
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


#ifndef COMTHREAD_H
#define COMTHREAD_H

#include <QThread>
#include <QImage>
#include <ros/service_client.h>
#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>

#include <re_kinect_object_detector/DetectionResult.h>

/**
 * Main ROS communication thread.
 **/
class ComThread : public QThread
{
    Q_OBJECT
public:
    explicit ComThread(QObject *parent = 0);

    /**
     * Image callback.
     **/
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    /**
     * Callback for detected objects.
     **/
    void kinect_detected_objectCb(const re_kinect_object_detector::DetectionResultConstPtr& msg);

signals:
    void updateZaragozaDetectionImg(QImage);
    /**
     * Gets called when a new detection image should be displayed.
     * @param QImage the image to display
     **/
    void updateKinectDetectionImg(QImage);

public slots:
    /**
     * Publishes a new model to the detection services.
     * @param model_dir directory containing the model
     * @param model_type type of the model
     * @param model_name name of the model
     **/
    void publishModelDir(QString model_dir, QString model_type, QString model_name);

protected:
    void run();

    ros::NodeHandle nh;

    ros::ServiceClient zaragoza_client;
    ros::Publisher zaragoza_model_dir_pub;

    ros::Publisher kinect_model_dir_pub;
    ros::Subscriber kinect_detected_objects_sub;

    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub;

    std::vector<std::string> zaragoza_model_names;
};

#endif // COMTHREAD_H
