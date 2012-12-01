/** \file slam_main.cpp
 * \brief starts the program's main loop
 *
 * This file is part of the RoboEarth ROS re_kinect_object_detector package.
 *
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by <a href="mailto:andreas.koch@ipvs.uni-stuttgart.de">Andreas Koch</a>, University of Stuttgart
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \author Andreas Koch
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 ***********************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <cv_bridge/CvBridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <highgui.h>
#include <cv.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/mutex.hpp>
#include <pcl/features/feature.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>

#include "re_kinect_object_detector/DetectionResult.h"

#include "recognitionmodel.h"

/// convert libEigen matrix to a tf::Transform
tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  btMatrix3x3 btm;
  btm.setValue(trans(0,0),trans(0,1),trans(0,2),
             trans(1,0),trans(1,1),trans(1,2),
             trans(2,0),trans(2,1),trans(2,2));
  btTransform ret;
  ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
  ret.setBasis(btm);
  return ret;
}

/**
 * Main class.
 **/
class ROSCom {
public:
    ROSCom() {
        ros::NodeHandle nh;
        kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points",1,
                                                            boost::bind(&ROSCom::kinect_cb,this,_1));
        features_pub = nh.advertise<sensor_msgs::PointCloud2>("re_kinect/feature_pcl",10);

        model_sub = nh.subscribe<std_msgs::String>("re_kinect/model_path", 1000, boost::bind(&ROSCom::model_path_cb, this, _1));

        detected_objects_pub = nh.advertise<re_kinect_object_detector::DetectionResult>("re_kinect/detection_results", 10);
    }

    /// callback for new model directories
    void model_path_cb(const std_msgs::StringConstPtr& model_path_msg) {
        boost::mutex::scoped_lock lock(mutex);
        ROS_INFO("model path callback; path = %s", model_path_msg->data.c_str());

        std::string model_path = model_path_msg->data;
        if (models.find(model_path) != models.end()) {
            ROS_WARN("model path '%s' was already known. Ignoring.", model_path.c_str());
            return;
        }

        RecognitionModel model;
        if (model.loadFromPath(model_path))
            models.insert(std::make_pair(model_path, model));

        ROS_INFO("done loading model from %s", model_path.c_str());
    }

    /// callback for Kinect point clouds
    void kinect_cb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg) {
        boost::mutex::scoped_lock lock(mutex);

        if (models.empty()) {
            static bool first_call = true;
            if (first_call) {
                ROS_WARN("no model files loaded!");
                first_call = false;
            }
            return;
        }

        // convert to PCL and image
        PointCloud cloud_;

        pcl::fromROSMsg(*pcl_msg, cloud_);

        sensor_msgs::ImagePtr image_(new sensor_msgs::Image);

        pcl::toROSMsg (cloud_, *image_);
        sensor_msgs::CvBridge bridge;

        IplImage* capture_ = bridge.imgMsgToCv(image_,std::string("bgr8"));
        ROS_INFO("received point cloud");

        cv::Mat image = cv::Mat(capture_,true);


        // Generate an aspect on the scene
        ObjectAspect scene;

        scene.calculate(image,cloud_.makeShared());

        Eigen::Matrix4f t;

        PointCloud feature_cloud;
        sensor_msgs::PointCloud2 feature_msg;
        int i = 0;
        re_kinect_object_detector::DetectionResult resultMsg;
        for(std::map<std::string, RecognitionModel>::iterator it=models.begin(); it!=models.end(); it++, i++) {
            RecognitionModel& model = it->second;
            re_msgs::DetectedObject detectedObjMsg;

            // Match the scene with the model.
            if (model.matchAspects(scene, t)) {

                // get transformation
                ROS_INFO("displaying %d points in frame: %s",(int)scene.match->points->size(),pcl_msg->header.frame_id.c_str());

                PointCloud cloudtransformed;
                pcl::transformPointCloud(*scene.match->points, cloudtransformed, t);

                if (i == 0)
                    feature_cloud = cloudtransformed;
                else
                    feature_cloud += cloudtransformed;

                for (size_t j=0 ; j< cloudtransformed.points.size(); j++ ){
                    PointType pt = cloudtransformed.points.at(j);
                    if (!isnan(pt.x)){
                        int y = pt.y*525.0 / pt.z + image.size().height / 2;
                        int x = pt.x*525.0 / pt.z + image.size().width / 2;

                        re_msgs::Pixel px;
                        px.x = x;
                        px.y = y;
                        detectedObjMsg.points2d.push_back(px);

                        geometry_msgs::Point threeD_pt;
                        threeD_pt.x = pt.x;
                        threeD_pt.y = pt.y;
                        threeD_pt.z = pt.z;

                        detectedObjMsg.points3d.push_back(threeD_pt);
                    }
                }
                tf::Transform object_tf = tfFromEigen(t);
                tf_broadcaster.sendTransform(tf::StampedTransform(object_tf, pcl_msg->header.stamp, pcl_msg->header.frame_id, model.model_name));

                resultMsg.ObjectNames.push_back(model.model_name);
                resultMsg.Detections.push_back(detectedObjMsg);

            }
            cv_bridge::CvImage out_msg;
            out_msg.header = pcl_msg->header;
            out_msg.encoding = "bgr8";
            out_msg.image = image;

            out_msg.toImageMsg(resultMsg.Image);
            detected_objects_pub.publish(resultMsg);
        }
        pcl::toROSMsg(feature_cloud,feature_msg);

        feature_msg.header.frame_id = pcl_msg->header.frame_id;
        feature_msg.header.stamp = pcl_msg->header.stamp;

        features_pub.publish(feature_msg);
    }


    ros::Subscriber kinect_sub;
    ros::Subscriber model_sub;
    ros::Publisher features_pub;
    ros::Publisher detected_objects_pub;
    tf::TransformBroadcaster tf_broadcaster;
    boost::mutex mutex;

    /// maps model paths to model objects (for checking if a given model path was already loaded)
    std::map<std::string, RecognitionModel> models;
};


int main(int argc, char* argv[]) {

    ros::init(argc,argv,"exec");

    ROSCom roscom;

#ifdef DEBUG_VIS
    cv::startWindowThread();
#endif
    ros::spin();


    return 0;
}
