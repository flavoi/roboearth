/** \file ar_kinect_extractobject.cpp
 * \brief Extracts the object inside a bounding box defined by markers
 *
 * In this file, a bounding box is created around the /PATTERN_BASE coordinate frame.
 * Given a point cloud from the Kinect camera, the point cloud inside this bounding box is extracted and
 * published to a predefined topic.
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

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/boundary.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/normal_3d.h>

#include "ar_kinect/ar_kinect.h"

using namespace std;


#ifdef AR_BOUNDING_BOX
// filter point that are outside a box, i.e. parallel to the main axes and defined by minp and maxp.
template<class PointType>
void cropBox(const pcl::PointCloud<PointType>& input, pcl::PointXYZ minp, pcl::PointXYZ maxp, pcl::PointCloud<PointType>& output) {
    //        pcl::PointIndicesPtr indicesPtr(new pcl::PointIndices);
    DBG(cout << "points in cloud: " << input.points.size() << endl;);

    pcl::PassThrough<PointType> passX;
    passX.setKeepOrganized(true);
    passX.setFilterFieldName("x");
    passX.setFilterLimits(minp.x,maxp.x);

    pcl::PassThrough<PointType> passY;
    passY.setKeepOrganized(true);
    passY.setFilterFieldName("y");
    passY.setFilterLimits(minp.y,maxp.y);

    pcl::PassThrough<PointType> passZ;
    passZ.setKeepOrganized(true);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits(minp.z,maxp.z);

    pcl::PointCloud<PointType> fX;
    pcl::PointCloud<PointType> fY;
    pcl::PointCloud<PointType> fZ;

    passX.setInputCloud(input.makeShared());
    passX.filter(fX);
    DBG(cout << "points left after x filter: " << fX.size() << endl;);

    passY.setInputCloud(fX.makeShared());
    passY.filter(fY);
    DBG(cout << "points left after y filter: " << fY.size() << endl;);
    passZ.setInputCloud(fY.makeShared());
    passZ.filter(fZ);

    output = fZ;
    DBG(cout << "points left after z filter: " << output.points.size() << endl;);
}

inline Eigen::Affine3f rosPoseToAffine(const geometry_msgs::PoseStamped& rosPose) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.translate(Eigen::Vector3f(rosPose.pose.position.x,
                rosPose.pose.position.y,
                rosPose.pose.position.z));

    transform.rotate(Eigen::Quaternionf(rosPose.pose.orientation.w,
                rosPose.pose.orientation.x,
                rosPose.pose.orientation.y,
                rosPose.pose.orientation.z));
    return transform;
}
#endif

void ARPublisher::extractObject(const sensor_msgs::PointCloud2ConstPtr& msg) {
#ifdef AR_BOUNDING_BOX
    ros::Time now = msg->header.stamp;

    geometry_msgs::PoseStamped origin;
    origin.header.frame_id = ORIGIN;
    origin.header.stamp = now;
    origin.pose.orientation.w = 1.f;

    pcl::PointCloud<pcl::PointXYZRGB> pcloud_in;
    pcl::fromROSMsg(*msg, pcloud_in);


    // get transformations
    geometry_msgs::PoseStamped cameraToMarkerPose;
    if (!tf_.waitForTransform(TARGET, ORIGIN, now, ros::Duration(0.5))) {
        ROS_INFO("no transformation from %s to %s. waiting...", ORIGIN, TARGET);
        return;
    }

    tf_.transformPose(TARGET, origin, cameraToMarkerPose);

    DBG(cout << "transformation from camera to marker: " << cameraToMarkerPose << endl;);
    Eigen::Affine3f cameraToMarkerTransform = rosPoseToAffine(cameraToMarkerPose);
    Eigen::Affine3f markerToCameraTransform = cameraToMarkerTransform.inverse();

    ros::param::get("boxsize", boundingbox_size);

    DBG(cout << "boxsize: " << boundingbox_size << endl;);

    pcl::PointXYZ minp(-boundingbox_size,-boundingbox_size,.01);
    pcl::PointXYZ maxp(boundingbox_size,boundingbox_size,2*boundingbox_size+.01);

    pcl::PointCloud<pcl::PointXYZ> boundingboxpcl_marker_frame;
    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(minp.x,minp.y,minp.z));
    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(minp.x,maxp.y,minp.z));
    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(minp.x,minp.y,maxp.z));
    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(minp.x,maxp.y,maxp.z));

    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(maxp.x,minp.y,minp.z));
    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(maxp.x,maxp.y,minp.z));
    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(maxp.x,minp.y,maxp.z));
    boundingboxpcl_marker_frame.push_back(pcl::PointXYZ(maxp.x,maxp.y,maxp.z));

    boundingboxpcl_marker_frame.header.frame_id = TARGET;

    pcl::PointCloud<pcl::PointXYZ> boundingboxpcl_camera_frame;
    pcl::transformPointCloud(boundingboxpcl_marker_frame, boundingboxpcl_camera_frame, markerToCameraTransform);


    // filter cloud with transformed bounding box
    pcl::PointXYZ minp_transformed(FLT_MAX,FLT_MAX,FLT_MAX);
    pcl::PointXYZ maxp_transformed(FLT_MIN,FLT_MIN,FLT_MIN);
    int count = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = boundingboxpcl_camera_frame.begin(); it != boundingboxpcl_camera_frame.end(); ++it ) {

        DBG(cerr << "Point " << count++ << ": " << it->x << "\t" <<it->y << "\t" << it->z  << endl;);

        if(it->x < minp_transformed.x) {
            minp_transformed.x = it->x;
        }
        if (it->x > maxp_transformed.x) {
            maxp_transformed.x = it->x;
        }

        if (it->y < minp_transformed.y) {
            minp_transformed.y = it->y;
        }
        if (it->y > maxp_transformed.y) {
            maxp_transformed.y = it->y;
        }

        if (it->z < minp_transformed.z) {
            minp_transformed.z = it->z;
        }
        if (it->z > maxp_transformed.z) {
            maxp_transformed.z = it->z;
        }
    }
    DBG(cout << minp_transformed << ", " << maxp_transformed << endl;);

    pcl::PointCloud<pcl::PointXYZRGB> pcloud_prefiltered;
    cropBox<pcl::PointXYZRGB>(pcloud_in, minp_transformed, maxp_transformed, pcloud_prefiltered);

    DBG(cout << "filtered " << pcloud_prefiltered.size()  - pcloud_in.size() << ", left: " << pcloud_prefiltered.size() << endl;);        


    // transform prefiltered pointcloud into TARGET coordinate system
    pcl::PointCloud<pcl::PointXYZRGB> pcloud_prefiltered_transformed;
    pcl::transformPointCloud(pcloud_prefiltered, pcloud_prefiltered_transformed, cameraToMarkerTransform);
    pcloud_prefiltered_transformed.header.frame_id = TARGET;
    // filter it according to original bounding box
    pcl::PointCloud<pcl::PointXYZRGB> pcloud_filtered_transformed;
    cropBox<pcl::PointXYZRGB>(pcloud_prefiltered_transformed, minp, maxp, pcloud_filtered_transformed);

    pcl::PointCloud<pcl::PointXYZRGB> pcloud_result;

#ifdef ACTIVATE_TABLE_FILTER
    // remove table surface
    // only remove surfaces in xz plane
    geometry_msgs::Vector3Stamped yvec_marker;
    yvec_marker.vector.x = 0;
    yvec_marker.vector.y = 1;
    yvec_marker.vector.z = 0;
    geometry_msgs::Vector3Stamped yvec_camera;
    yvec_marker.header.frame_id = TARGET;
    yvec_marker.header.stamp = now;
    listener.transformVector(ORIGIN, yvec_marker, yvec_camera);
    Eigen::Vector3f yvector_camera(yvec_camera.vector.x, yvec_camera.vector.y, yvec_camera.vector.z);
    DBG(cout << "y Vector: " << yvector_camera << endl;);


    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setAxis(yvector_camera);
    // Optional
    seg.setOptimizeCoefficients (false);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);

    double randist = 0.01;

    ros::param::get("RANSAC_DISTANCE",randist);

    seg.setDistanceThreshold (randist);

    seg.setInputCloud (pcloud_filtered_transformed.makeShared());
    seg.segment (*inliers, coefficients);
    DBG(cout << "table points: " << inliers->get_indices_size() << endl;);

    pcl::ExtractIndices<pcl::PointXYZRGB> extractIndices;
    extractIndices.setInputCloud(pcloud_prefiltered.makeShared());
    extractIndices.setIndices(inliers);
    extractIndices.setNegative(true);
    extractIndices.filter(pcloud_result);


#else

    // make sure there are valid points
    bool validdata = false;
    for(size_t i=0; i<pcloud_filtered_transformed.points.size(); i++) {
        if (!isnan(pcloud_filtered_transformed.points.at(i).x)) {
            validdata = true;
            break;
        }
    }
    if (!validdata) {
        ROS_WARN("no point cloud found inside the bounding box above the marker pattern.");
        return;
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr flann(new pcl::search::KdTree<pcl::PointXYZRGB>);

    cerr << "estimating normals" << endl;
    clock_t start = clock();

    pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(flann);
    norm_est.setInputCloud(pcloud_filtered_transformed.makeShared());
    norm_est.setRadiusSearch(0.01);
    norm_est.compute(*normals_cloud);

    cerr << (double)(clock() - start) / (double)CLOCKS_PER_SEC << " seconds needed for normal estimation" << endl;

    /*
    // filter out boundary points
    start = clock();
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> boundary_est;
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary>::PointCloudOut boundary;
    boundary_est.setRadiusSearch(0.03);
    boundary_est.setInputNormals(normals_cloud);

    boundary_est.setSearchMethod(flann);
    boundary_est.setInputCloud(pcloud_filtered_transformed.makeShared());

    boundary_est.angle_threshold_ = CV_PI / 6;
    boundary_est.compute(boundary);

    for(int i = 0; i < boundary.size(); i++) {
        if (boundary.points[i].boundary_point == 0) {
            pcloud_result.push_back(pcloud_filtered_transformed.points.at(i));
        } else {
            pcl::PointXYZRGB pt = pcloud_filtered_transformed.points.at(i);
            pt.x = NAN;
            pt.y = NAN;
            pt.z = NAN;
            pcloud_result.push_back(pt);
        }
    }


    cerr << pcloud_filtered_transformed.size() << "\t" << pcloud_result.size() << endl;
    cerr << (double)(clock() - start) / (double)CLOCKS_PER_SEC << " seconds needed for normal filter" << endl;
*/
    pcloud_result = pcloud_filtered_transformed;
#endif

    sensor_msgs::PointCloud2 result_msg;

    pcl::toROSMsg<pcl::PointXYZRGB>(pcloud_result, result_msg);

    result_msg.header.frame_id = TARGET;
    result_msg.header.stamp = now;
    result_pcl.publish(result_msg);

#endif

}
