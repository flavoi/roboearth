/** \file ar_kinect.cpp
 * \brief Detect markers and calculate object coordinate system.
 *
 * Contains the package's main execution loop.
 * Based on Michael Ferguson's ar_kinect.
 *
 * This file is part of the RoboEarth ROS ar_bounding_box package.
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
 * \author Daniel Di Marco
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 ***********************************************/

#include <iostream>

#include <time.h>
#include <Eigen/Eigen>
#include <Eigen/LU>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <math.h>
#include <cv_bridge/CvBridge.h>
#include <tf/transform_listener.h>

#include "CoordinateFrame.h"

#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>

#include "ar_kinect/ar_kinect.h"
#include "ar_kinect/object.h"

using namespace std;

/// helper function to convert a opencv 3d point to pcl::PointXYZ
inline pcl::PointXYZ getPointPCLFromCV(CvPoint3D32f pcv) {
    pcl::PointXYZ p;
    p.x = pcv.x;
    p.y = pcv.y;
    p.z = pcv.z;
    return p;
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "ar_kinect");
    ros::NodeHandle n;
    ARPublisher ar_kinect (n);

    ros::spin ();

    return 0;
}


#ifdef AR_BOUNDING_BOX
/// Sort markers by quality utility function.
bool compareDistances(const ar_pose::ARMarker& a1, const ar_pose::ARMarker& a2) {
    double dist1 = sqrt(a1.pose.pose.position.x * a1.pose.pose.position.x + a1.pose.pose.position.y * a1.pose.pose.position.y + a1.pose.pose.position.z * a1.pose.pose.position.z);
    double dist2 = sqrt(a2.pose.pose.position.x * a2.pose.pose.position.x + a2.pose.pose.position.y * a2.pose.pose.position.y + a2.pose.pose.position.z * a2.pose.pose.position.z);
    return dist1 < dist2;
}

#endif

    ARPublisher::ARPublisher (ros::NodeHandle & n):n_ (n), it_ (n_), gotcloud_(false), transformations(boost::extents[6][6][6])
    {
        std::string path;
        std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
        ros::NodeHandle n_param ("~");
        XmlRpc::XmlRpcValue xml_marker_center;
        cloud_width_ = 640;

        // **** get parameters

        if (!n_param.getParam ("publish_tf", publishTf_))
            publishTf_ = true;
        ROS_INFO ("\tPublish transforms: %d", publishTf_);

        if (!n_param.getParam ("publish_visual_markers", publishVisualMarkers_))
            publishVisualMarkers_ = true;
        ROS_INFO ("\tPublish visual markers: %d", publishVisualMarkers_);

        if (!n_param.getParam ("threshold", threshold_))
            threshold_ = 100;
        ROS_INFO ("\tThreshold: %d", threshold_);

        if (!n_param.getParam ("marker_pattern_list", path)){
            sprintf(pattern_filename_, "%s/data/objects_kinect", package_path.c_str());
        }else{
            sprintf(pattern_filename_, "%s", path.c_str());
        }
        ROS_INFO ("Marker Pattern Filename: %s", pattern_filename_);

        if (!n_param.getParam ("marker_data_directory", path)){
            sprintf(data_directory_, "%s", package_path.c_str());
        }else{
            sprintf(data_directory_, "%s", path.c_str());
        }
        ROS_INFO ("Marker Data Directory: %s", data_directory_);

        // **** subscribe

        ROS_INFO ("Subscribing to info topic");
        sub_ = n_.subscribe (cameraInfoTopic_, 1, &ARPublisher::camInfoCallback, this);
        cloud_sub_ = n_.subscribe(cloudTopic_, 1, &ARPublisher::cloudCallback, this);

        getCamInfo_ = false;

        // **** advertise

        arMarkerPub_ = n_.advertise < ar_pose::ARMarkers > ("ar_pose_markers",0);
        if(publishVisualMarkers_)
        {
            rvizMarkerPub_ = n_.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
        }

#ifdef DEBUG_AR_KINECT
        kinect_pclPub_ = n_.advertise < sensor_msgs::PointCloud2 > ("debug_kinect_pcl",1);
#endif
    }

    ARPublisher::~ARPublisher (void)
    {
        arVideoCapStop ();
        arVideoClose ();
    }

    void ARPublisher::camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
    {
        if (!getCamInfo_)
        {
            cam_info_ = (*cam_info);

            cam_param_.xsize = cam_info_.width;
            cam_param_.ysize = cam_info_.height;

            cam_param_.mat[0][0] = cam_info_.P[0];
            cam_param_.mat[1][0] = cam_info_.P[4];
            cam_param_.mat[2][0] = cam_info_.P[8];
            cam_param_.mat[0][1] = cam_info_.P[1];
            cam_param_.mat[1][1] = cam_info_.P[5];
            cam_param_.mat[2][1] = cam_info_.P[9];
            cam_param_.mat[0][2] = cam_info_.P[2];
            cam_param_.mat[1][2] = cam_info_.P[6];
            cam_param_.mat[2][2] = cam_info_.P[10];
            cam_param_.mat[0][3] = cam_info_.P[3];
            cam_param_.mat[1][3] = cam_info_.P[7];
            cam_param_.mat[2][3] = cam_info_.P[11];

            cam_param_.dist_factor[0] = cam_info_.K[3];       // x0 = cX from openCV calibration
            cam_param_.dist_factor[1] = cam_info_.K[6];       // y0 = cY from openCV calibration
            cam_param_.dist_factor[2] = -100.*cam_info_.D[0];  // f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
            cam_param_.dist_factor[3] = 1.0;                  // scale factor, should probably be >1, but who cares...

            arInit ();

            ROS_INFO ("Subscribing to image and cloud topics");

            getCamInfo_ = true;
        }
    }

    void ARPublisher::arInit ()
    {
        arInitCparam (&cam_param_);
        ROS_INFO ("*** Camera Parameter ***");
        arParamDisp (&cam_param_);

        // load in the object data - trained markers and associated bitmap files
        if ((object = ar_object::read_ObjData (pattern_filename_, data_directory_, &objectnum)) == NULL)
            ROS_BREAK ();
        ROS_DEBUG ("Objectfile num = %d", objectnum);

        sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
        capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 3);

#ifdef ACTIVATE_TABLE_FILTER
        ros::param::set("RANSAC_DISTANCE",0.01);
#endif

        DBG(cerr << "building.";);
#ifdef AR_BOUNDING_BOX
        result_pcl = n_.advertise<sensor_msgs::PointCloud2>("resulting_pcl", 1000);
        bb_pcl = n_.advertise<sensor_msgs::PointCloud2>("bb_pcl", 1000);

        if (!ros::param::has("boxsize"))
            ros::param::set("boxsize", 0.135);
        ros::param::param<double>("boxsize", boundingbox_size, 0.135);
        if (!ros::param::has("marker_dist_threshold"))
            ros::param::set("marker_dist_threshold", 0.002);
        ros::param::param<double>("marker_dist_threshold", marker_dist_threshold, .002);

        const float size = .11;

        // If you want to use a different marker patter layout
        // specify here the vector from the marker center to the
        // bounding box center.
        CvPoint3D32f ps[6] = {cvPoint3D32f(size, -size, 0),
                              cvPoint3D32f(0, -size, 0),
                              cvPoint3D32f(-size, 0, 0),
                              cvPoint3D32f(-size, size, 0),
                              cvPoint3D32f(0, size, 0),
                              cvPoint3D32f(size, 0, 0)};

        // Calculate the resulting transforms and distance look up table.
         distances = cv::Mat1f(6,6);

        for(int i=0; i<6; i++) {
            for (int j=0; j<6; j++) {
                distances(cv::Point(i,j)) = Utils3D::distPoints(&ps[i],&ps[j]);
                for (int k=0; k<6; k++) {
                    CvPoint3D32f p1, p2, p3;
                    p1 = ps[i];
                    p2 = ps[j];
                    p3 = ps[k];

                    transformations[i][j][k] = new CoordinateFrame(&p1, &p2, &p3);
                }
            }
        }
#endif
    }

    void ARPublisher::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        bool published = false;

        // convert to PCL
        pcl::fromROSMsg(*msg, cloud_);

        // can now use clouds
        gotcloud_ = true;

        sensor_msgs::ImagePtr image_(new sensor_msgs::Image);

        pcl::toROSMsg (cloud_, *image_);
        sensor_msgs::CvBridge bridge;

        capture_ = bridge.imgMsgToCv(image_,std::string("bgr8"));

#if 0
        // preprocess the input image
        IplImage* gray = cvCreateImage(cvSize(640,480),8,1);
        cvCvtColor(capture_, gray, CV_BGR2GRAY);
//        cvNamedWindow("orig");
//        cvShowImage("orig", gray);
        cvThreshold(gray, gray, 128, 255, CV_THRESH_BINARY);
        cvSmooth(gray, gray, CV_GAUSSIAN, 5, 5);

        cvCvtColor(gray, capture_, CV_GRAY2BGR);
//        cvNamedWindow("win");
//        cvShowImage("win",capture_);
        cvWaitKey(10);
        cvReleaseImage(&gray);
#endif

        ARUint8 *dataPtr;
        ARMarkerInfo *marker_info;
        int marker_num;
        int i, k, j;

        dataPtr = (ARUint8 *) capture_->imageData;

        // detect the markers in the video frame
        if (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0)
        {
            argCleanup ();
            ROS_BREAK ();
        }

        if (marker_num < 3) {
            ROS_WARN("detected %i markers in the image; at least 3 markers are necessary to estimate a coordinate system. please check your camera setup.", marker_num);
            return;
        }

#ifdef DEBUG_AR_KINECT
        pcl::PointCloud<pcl::PointXYZRGB> pcl_dat;
        sensor_msgs::PointCloud2 pcl_msg;
#endif

        int downsize = capture_->width/cloud_width_;

        arPoseMarkers_.markers.clear ();
        // check for known patterns
        for (i = 0; i < objectnum; i++) {
            k = -1;
            for (j = 0; j < marker_num; j++) {
                if (object[i].id == marker_info[j].id) {
                    if (k == -1)
                        k = j;
                    else {
                        // make sure you have the best pattern (highest confidence factor)
                        if (marker_info[k].cf < marker_info[j].cf)
                            k = j;
                    }
                }
            }
            if (k == -1) {
                object[i].visible = 0;
                continue;
            }

            // **** these are in the ROS frame
            double quat[4], pos[3];

            if (object[i].visible == 0)
            {
                arGetTransMat (&marker_info[k], object[i].marker_center, object[i].marker_width, object[i].trans);
            }
            else
            {
                arGetTransMatCont (&marker_info[k], object[i].trans,
                                   object[i].marker_center, object[i].marker_width, object[i].trans);
            }
            object[i].visible = 1;

            double arQuat[4], arPos[3];
            arUtilMat2QuatPos (object[i].trans, arQuat, arPos);

            pos[0] = arPos[0] * AR_TO_ROS;
            pos[1] = arPos[1] * AR_TO_ROS;
            pos[2] = arPos[2] * AR_TO_ROS;

            quat[0] = -arQuat[0];
            quat[1] = -arQuat[1];
            quat[2] = -arQuat[2];
            quat[3] = arQuat[3];

            if(gotcloud_){
                // Do high-definition via point clouds!

                pcl::PointXYZRGB point = cloud_((int)  (marker_info[k].pos[0]/downsize), (int) (marker_info[k].pos[1]/downsize));

                if(!(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))) {

                    // Get the correct order (orientation) of the corner points
                    int idx1 = (4 - marker_info[k].dir) + 0;
                    int idx2 = (4 - marker_info[k].dir) + 1;
                    int idx3 = (4 - marker_info[k].dir) + 2;

                    idx1 %= 4;
                    idx2 %= 4;
                    idx3 %= 4;

                    pcl::PointXYZRGB p1, p2, p3;
                    {
                        int u1 = static_cast<int>(marker_info[k].vertex[idx1][0]);
                        int v1 = static_cast<int>(marker_info[k].vertex[idx1][1]);
                        int u2 = static_cast<int>(marker_info[k].vertex[idx2][0]);
                        int v2 = static_cast<int>(marker_info[k].vertex[idx2][1]);
                        int u3 = static_cast<int>(marker_info[k].vertex[idx3][0]);
                        int v3 = static_cast<int>(marker_info[k].vertex[idx3][1]);

                        if ((u1 < 0) || (u1 >= cloud_.width) || (v1 < 0) || (v1 >= cloud_.height) ||
                                (u2 < 0) || (u2 >= cloud_.width) || (v2 < 0) || (v2 >= cloud_.height) ||
                                (u3 < 0) || (u3 >= cloud_.width) || (v3 < 0) || (v3 >= cloud_.height)) {
                            ROS_WARN("got invalid corner points from ar_bounding_box");
                            return;
                        }

                        p2 = cloud_.at(u1,v1);
                        p1 = cloud_.at(u2,v2);
                        p3 = cloud_.at(u3,v3);
                    }
                    cv::Point2f p2_(marker_info[k].vertex[idx1][0], marker_info[k].vertex[idx1][1]);
                    cv::Point2f p1_(marker_info[k].vertex[idx2][0], marker_info[k].vertex[idx2][1]);
                    cv::Point2f p3_(marker_info[k].vertex[idx3][0], marker_info[k].vertex[idx3][1]);

                    // define a cross that cuts the marker in four squares.
                    cv::Vec2f v1 = p2_ - p1_;
                    cv::Vec2f v2 = p3_ - p1_;

                    vector<cv::Point3f> l1;

                    cv::Point2f m1 = cv::Point2f((p1_.x+ p3_.x)*.5,(p1_.y+ p3_.y)*.5);
                    cv::Point2f m2 = cv::Point2f((p1_.x+ p2_.x)*.5,(p1_.y+ p2_.y)*.5);


                    // Get the corresponding depth information (x,y,z)
                    // TODO: Might be better to use bresenham here.
                    for (double k1 = 0; k1 < 1; k1+=.01) {
                        cv::Point2f tpos = m1  + cv::Point2f(k1* v1[0],k1*v1[1]);
                        if (tpos.x >= 0 && tpos.x < cloud_.width && tpos.y >= 0 && tpos.y < cloud_.height) {
                            pcl::PointXYZRGB tpoint = cloud_.at((int)tpos.x,(int)tpos.y);
                            if (!isnan(tpoint.x))
                                l1.push_back(cv::Point3f(tpoint.x,tpoint.y,tpoint.z));
                        }
                    }

                    vector<cv::Point3f> l2;

                    // TODO: Might be better to use bresenham here.
                    for (double k2 = 0; k2 < 1; k2+=.01) {
                        cv::Point2f tpos = m2    + cv::Point2f(k2* v2[0],k2*v2[1]);
                        //                    cout << "2: " << tpos << endl;
                        if (tpos.x >= 0 && tpos.x < cloud_.width && tpos.y >= 0 && tpos.y < cloud_.height) {
                            pcl::PointXYZRGB tpoint = cloud_.at((int)tpos.x,(int)tpos.y);
                            if (!isnan(tpoint.x))
                                l2.push_back(cv::Point3f(tpoint.x,tpoint.y,tpoint.z));
                        }
                    }

                    // If we received enough points...
                    if (l1.size() >= 6 && l2.size() >= 6) {

                        // ... we can least square fit a line to the point series
                        cv::Vec6f line2_parameters;
                        cv::fitLine(cv::Mat(l2),line2_parameters,CV_DIST_L2,0.01,0.01,0.01);

                        cv::Vec6f line1_parameters;
                        cv::fitLine(cv::Mat(l1),line1_parameters,CV_DIST_L2,0.01,0.01,0.01);

                        cv::Point3f v12;
                        cv::Point3f p12;
                        v12.x = line1_parameters[0];
                        v12.y = line1_parameters[1];
                        v12.z = line1_parameters[2];

                        p12.x = line1_parameters[3];
                        p12.y = line1_parameters[4];
                        p12.z = line1_parameters[5];

                        cv::Point3f v13;
                        cv::Point3f p13;
                        v13.x = line2_parameters[0];
                        v13.y = line2_parameters[1];
                        v13.z = line2_parameters[2];

                        p13.x = line2_parameters[3];
                        p13.y = line2_parameters[4];
                        p13.z = line2_parameters[5];


                        // Copy directions and locations of the lines to
                        // perform further computations

                        Utils3D::STRAIGHTLINE line1;
                        line1.loc = p12;
                        line1.dir = v12;

                        Utils3D::STRAIGHTLINE line2;
                        line2.loc = p13;
                        line2.dir = v13;

                        cv::Point3f v1__ = l1[0] - l1[l1.size()-1];

                        if (v1__.ddot(line1.dir) < 0)
                        {
                            Utils3D::scalePoint(&line1.dir,-1);
                        }

                        cv::Point3f v2__ = l2[0] - l2[l2.size()-1];

                        // Maybe the direction was toggled when performing least square line fit.
                        // restore it, using the first and last point from the sequence.
                        if (v2__.ddot(line2.dir) < 0)
                        {
                            Utils3D::scalePoint(&line2.dir,-1);
                        }

                        float t1,t2;

                        // Calculate the marker center by finding the shortest distance (and the two
                        // corresponding points) and calculate the point in between
                        float distance = Utils3D::getShortestDistanceBetweenTwoLines(line1,line2,&t1,&t2);

                        if (distance > 1) {
                            cerr << "the two lines do not meet exactly." << endl;
                        } else {
                            CvPoint3D32f p_1_ = Utils3D::getLinePoint(line1,t1);
                            CvPoint3D32f p_2_ = Utils3D::getLinePoint(line2,t2);

                            //                        cerr << "t1/t2:" << t1 << "\t" << t2 << endl;

                            CvPoint3D32f tpt;
                            CV_SWAP(line1.loc,line2.loc,tpt);

                            // We are now able to calculate the three points that define or marker base
                            // based on depth information.
                            p1.x = (p_1_.x + p_2_.x)*.5;
                            p1.y = (p_1_.y + p_2_.y)*.5;
                            p1.z = (p_1_.z + p_2_.z)*.5;

                            p12 = Utils3D::getLinePoint(line1,1);
                            p13 = Utils3D::getLinePoint(line2,1);

                            p2.x = p12.x;
                            p2.y = p12.y;
                            p2.z = p12.z;

                            p3.x = p13.x;
                            p3.y = p13.y;
                            p3.z = p13.z;
                        }
                    } else {
                        cerr << "error, too few points: " << l1.size() << "\t"<<l2.size() << endl;
                        marker_info[i].cf = 0;
                    }

#ifdef DEBUG_AR_KINECT
                    pcl_dat.points.push_back(p1);
                    pcl_dat.points.push_back(p2);
                    pcl_dat.points.push_back(p3);


                    for (size_t pidx = 0; pidx < l1.size(); pidx++) {
                        pcl::PointXYZRGB tp;
                        tp.x = l1[pidx].x;
                        tp.y = l1[pidx].y;
                        tp.z = l1[pidx].z;
                        pcl_dat.push_back(tp);
                    }

                    for (size_t pidx = 0; pidx < l2.size(); pidx++) {
                        pcl::PointXYZRGB tp;
                        tp.x = l2[pidx].x;
                        tp.y = l2[pidx].y;
                        tp.z = l2[pidx].z;
                        pcl_dat.push_back(tp);
                    }
#endif


                    // Construct the marker base. Use the convenience functions
                    CoordinateFrame e6p(cv::Point3f(p1.x, p1.y, p1.z), cv::Point3f(p2.x, p2.y, p2.z), cv::Point3f(p3.x, p3.y, p3.z));
                    btQuaternion q(e6p.getRot()->z,e6p.getRot()->y,e6p.getRot()->x);

                    pos[0] = p1.x;
                    pos[1] = p1.y;
                    pos[2] = p1.z;

                    quat[0] = (double) q.x();
                    quat[1] = (double) q.y();
                    quat[2] = (double) q.z();
                    quat[3] = (double) q.w();
                }
            }

            // **** publish the marker

            ar_pose::ARMarker ar_pose_marker;
            ar_pose_marker.header.frame_id = msg->header.frame_id;
            ar_pose_marker.header.stamp = msg->header.stamp;
            ar_pose_marker.id = object[i].id;

            ar_pose_marker.pose.pose.position.x = pos[0];
            ar_pose_marker.pose.pose.position.y = pos[1];
            ar_pose_marker.pose.pose.position.z = pos[2];

            ar_pose_marker.pose.pose.orientation.x = quat[0];
            ar_pose_marker.pose.pose.orientation.y = quat[1];
            ar_pose_marker.pose.pose.orientation.z = quat[2];
            ar_pose_marker.pose.pose.orientation.w = quat[3];

            ar_pose_marker.confidence = marker_info[i].cf * 100.0f;
            arPoseMarkers_.markers.push_back (ar_pose_marker);

            // **** publish transform between camera and marker

            btQuaternion rotation (quat[0], quat[1], quat[2], quat[3]);
            btVector3 origin (pos[0], pos[1], pos[2]);
            btTransform t (rotation, origin);

            if (publishTf_)
            {
                tf::StampedTransform camToMarker (t, msg->header.stamp, msg->header.frame_id, object[i].name);
                broadcaster_.sendTransform(camToMarker);
            }

            // **** publish visual marker

            if (publishVisualMarkers_)
            {
                btVector3 markerOrigin (0, 0, 0.25 * object[i].marker_width * AR_TO_ROS);
                btTransform m (btQuaternion::getIdentity (), markerOrigin);
                btTransform markerPose = t * m; // marker pose in the camera frame

                tf::poseTFToMsg (markerPose, rvizMarker_.pose);

                rvizMarker_.header.frame_id = msg->header.frame_id;
                rvizMarker_.header.stamp = msg->header.stamp;
                rvizMarker_.id = object[i].id;

                rvizMarker_.scale.x = 1.0 * object[i].marker_width * AR_TO_ROS;
                rvizMarker_.scale.y = 1.0 * object[i].marker_width * AR_TO_ROS;
                rvizMarker_.scale.z = 0.5 * object[i].marker_width * AR_TO_ROS;
                rvizMarker_.ns = "basic_shapes";
                rvizMarker_.type = visualization_msgs::Marker::CUBE;
                rvizMarker_.action = visualization_msgs::Marker::ADD;
                switch (i)
                {
                case 0:
                    rvizMarker_.color.r = 0.0f;
                    rvizMarker_.color.g = 0.0f;
                    rvizMarker_.color.b = 1.0f;
                    rvizMarker_.color.a = 1.0;
                    break;
                case 1:
                    rvizMarker_.color.r = 1.0f;
                    rvizMarker_.color.g = 0.0f;
                    rvizMarker_.color.b = 0.0f;
                    rvizMarker_.color.a = 1.0;
                    break;
                default:
                    rvizMarker_.color.r = 0.0f;
                    rvizMarker_.color.g = 1.0f;
                    rvizMarker_.color.b = 0.0f;
                    rvizMarker_.color.a = 1.0;
                }
                rvizMarker_.lifetime = ros::Duration ();

                rvizMarkerPub_.publish (rvizMarker_);
                ROS_DEBUG ("Published visual marker");
            }
        }
        arMarkerPub_.publish (arPoseMarkers_);
        ROS_DEBUG ("Published ar_multi markers");

#ifdef AR_BOUNDING_BOX

        if (arPoseMarkers_.markers.size() >= 3) {
            DBG(ROS_INFO("Found enough markers"););

            CoordinateFrame* transformFromModelPointsToObjectBase = NULL;

            CoordinateFrame transformFromScenePointsToCamera;

            int pointsfound = 0;
            CvPoint3D32f ps[3];
            int idxs[3];

            std::sort(arPoseMarkers_.markers.begin(),arPoseMarkers_.markers.end(),compareDistances);

            bool destroyresults = false;

            for (unsigned int i = 0; i < arPoseMarkers_.markers.size() && pointsfound < 3; i++) {

                idxs[pointsfound] = arPoseMarkers_.markers[i].id;
                ps[pointsfound] = cvPoint3D32f(arPoseMarkers_.markers[i].pose.pose.position.x,
                                               arPoseMarkers_.markers[i].pose.pose.position.y,
                                               arPoseMarkers_.markers[i].pose.pose.position.z);

                DBG(cerr << "ps[" << pointsfound << "]: " << cv::Point3f(ps[pointsfound]) << " quality: " << (float)arPoseMarkers_.markers[i].confidence << endl;);

                pointsfound++;
            }

            ros::param::get("marker_dist_threshold", marker_dist_threshold);
            double d01 = Utils3D::distPoints(&ps[0],&ps[1]);
            if ( fabs(d01 - distances(cv::Point(idxs[0],idxs[1])) ) > marker_dist_threshold) {
                destroyresults = true;
                cerr << "distance 0/1 too large: " << fabs(d01 - distances(cv::Point(idxs[0],idxs[1])) ) <<endl;
            }

            double d12 = Utils3D::distPoints(&ps[1],&ps[2]);
            if ( fabs(d12 - distances(cv::Point(idxs[1],idxs[2])) ) > marker_dist_threshold) {
                destroyresults = true;
                cerr << "distance 1/2 too large: " << fabs(d12 - distances(cv::Point(idxs[1],idxs[2])) ) <<endl;
            }

            double d02 = Utils3D::distPoints(&ps[0],&ps[2]);
            if ( fabs(d02 - distances(cv::Point(idxs[0],idxs[2])) ) > marker_dist_threshold) {
                destroyresults = true;
                cerr << "distance 2/0 too large: " <<  fabs(d02 - distances(cv::Point(idxs[0],idxs[2])) ) <<endl;
            }

            // check marker plausibility

            sensor_msgs::PointCloud2 bb_msg;
            pcl::PointCloud<pcl::PointXYZ> base_points_pcl;

            base_points_pcl.points.push_back(getPointPCLFromCV(ps[0]));
            base_points_pcl.points.push_back(getPointPCLFromCV(ps[1]));
            base_points_pcl.points.push_back(getPointPCLFromCV(ps[2]));
            pcl::toROSMsg(base_points_pcl, bb_msg);
            bb_msg.header.frame_id = ORIGIN;
            bb_msg.header.stamp = msg->header.stamp;
            bb_pcl.publish(bb_msg);

            transformFromModelPointsToObjectBase = transformations[idxs[0]][idxs[1]][idxs[2]];
            CvPoint3D32f bp1 = ps[0],bp2 = ps[1],bp3 = ps[2];
            transformFromScenePointsToCamera = CoordinateFrame(&bp1,&bp2,&bp3);
            if (transformFromModelPointsToObjectBase != NULL) {
                CvMat* trafo = transformFromModelPointsToObjectBase->createTransformationMatrix();
                cvInvert(trafo,trafo);
                CoordinateFrame transformFromObjectBaseToModelPoints(trafo);
                CoordinateFrame* e6p = transformFromObjectBaseToModelPoints.cInWorld(&transformFromScenePointsToCamera);

                DBG(cerr << e6p->toString() << endl;);

                btQuaternion rotation(e6p->getRot()->z,e6p->getRot()->y,e6p->getRot()->x);
                btVector3 origin (e6p->getPos()->x, e6p->getPos()->y, e6p->getPos()->z);            btTransform t (rotation, origin);

                broadcaster_.sendTransform(tf::StampedTransform(t,msg->header.stamp,ORIGIN, "PATTERN_BASE"));

                published = !destroyresults;
            }
        }
#endif

#ifdef DEBUG_AR_KINECT
        pcl::toROSMsg(pcl_dat,pcl_msg);
        pcl_msg.header.stamp = ros::Time::now();
        pcl_msg.header.frame_id = msg->header.frame_id;
        kinect_pclPub_.publish(pcl_msg);
#endif

#ifdef AR_BOUNDING_BOX
        if(published)
            extractObject(msg);
#endif
    }

