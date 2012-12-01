/** \file ar_kinect.h
 * \brief Main header file for ar_bounding box
 * 
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

#ifndef AR_POSE_AR_MULTI_H
#define AR_POSE_AR_MULTI_H

#include <string.h>
#include <stdarg.h>

#include <AR/ar.h>
#include <AR/arMulti.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>

#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>

#include <ar_pose/ARMarkers.h>
#include <ar_kinect/object.h>

#include <boost/multi_array.hpp>

#define AR_BOUNDING_BOX
#define DEBUG_AR_KINECT

/// Kinect camera origin tf frame
#define ORIGIN "/camera_rgb_optical_frame"
/// object center tf frame
#define TARGET "/PATTERN_BASE"

#define DBG(str) //str

const std::string cameraImageTopic_ = "/camera/depth_registered/image";
const std::string cameraInfoTopic_  = "/camera/depth_registered/camera_info";
const std::string cloudTopic_ = "/camera/depth_registered/points";


class CoordinateFrame;

const double AR_TO_ROS = 0.001;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class ARPublisher
{
    public:
        ARPublisher (ros::NodeHandle & n);
        ~ARPublisher (void);

    private:
        /// Loads the patterns & initializes the lookup tables.
        void arInit ();
        /// Camera parameters callback
        void camInfoCallback (const sensor_msgs::CameraInfoConstPtr &);
        /**
         * Callback for the point cloud from the Kinect.
         * Estimates the marker's positions and calculates the object coordinate system.
         **/
        void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg);
        /**
         * Crops the sensor's point cloud at the bounding box around the object coordinate system
         * and publishes it.
         **/
        void extractObject(const sensor_msgs::PointCloud2ConstPtr& msg);

        ros::NodeHandle n_;
        tf::TransformBroadcaster broadcaster_;
        ros::Subscriber sub_;
        image_transport::Subscriber cam_sub_;
        ros::Subscriber cloud_sub_;    
        ros::Publisher arMarkerPub_;
#ifdef DEBUG_AR_KINECT
        ros::Publisher kinect_pclPub_;
#endif

        image_transport::ImageTransport it_;
        sensor_msgs::CvBridge bridge_;
        sensor_msgs::CameraInfo cam_info_;
        PointCloud cloud_;

        // **** for visualisation in rviz
        ros::Publisher rvizMarkerPub_;
        visualization_msgs::Marker rvizMarker_;

        // **** parameters
        /// Camera Calibration Parameters
        ARParam cam_param_;
        /// AR Marker Info
        ARMultiMarkerInfoT *config;
        ar_object::ObjectData_T * object;
        int objectnum;
        char pattern_filename_[FILENAME_MAX];
        char data_directory_[FILENAME_MAX];

        ar_pose::ARMarkers arPoseMarkers_;
        int threshold_;
        bool getCamInfo_;
        bool publishTf_;
        bool publishVisualMarkers_;
        /// camera image size
        CvSize sz_;
        IplImage *capture_;
        bool gotcloud_;
        int cloud_width_;
        /// lookup table for distances between marker positions (for plausability checks)
        cv::Mat1f distances;
        /// size of the bounding box to crop
        double boundingbox_size;
        /// threshold for marker position accuracy
        double marker_dist_threshold;

        message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
        tf::TransformListener tf_;

        /// publisher for the cropped point cloud
        ros::Publisher result_pcl;
        /// publisher for the bounding box (for debugging)
        ros::Publisher bb_pcl;

        typedef boost::multi_array<CoordinateFrame*, 3> transform_type;
        /// Lookup table for getting the object coordinate system from 3 marker positions.
        transform_type transformations;

};                            // end class ARPublisher

#endif
