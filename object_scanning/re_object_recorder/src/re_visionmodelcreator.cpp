/** \file re_visionmodelcreator.cpp
 * \brief Creates object models for WP1's object recognition
 *
 * Tries to create object models from colored 3D point clouds that can be used by re_vision.
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

#include "re_visionmodelcreator.h"

#include <cv.h>
#include <highgui.h>
#include <boost/foreach.hpp>
#include <QString>
#include <QFile>
#include <QXmlStreamWriter>

#include <Eigen/Geometry>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>

#include <re_vision/SurfSet.h>
#include <re_vision/BundleCamera.h>
#include <re_vision/PLYFile.h>

using namespace std;

void re_visionModelCreator::createMetaFile(QDir model_dir, QString modelname, QString modeltype, size_t face_count, double scale) {
    QFile metaFile(model_dir.filePath("meta.xml"));
    metaFile.open(QIODevice::WriteOnly);
    QXmlStreamWriter xml(&metaFile);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();

    xml.writeStartElement("model");
    xml.writeTextElement("name", modelname);
    xml.writeTextElement("type", modeltype);
    xml.writeTextElement("faces", QString::number(face_count));
    xml.writeStartElement("dimensions");
    xml.writeTextElement("scale", QString::number(scale));
    xml.writeEndElement(); // dimensions
    xml.writeEndElement(); // model
    xml.writeEndDocument();
}

/// transformation from re_object_recorder's object coordinate system to re_vision's
Eigen::Affine3f getRe_visionTransformation() {
    Eigen::Quaternionf quat(Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitZ()));
//    transformation.setRotation(quat);
    Eigen::Affine3f transformation(quat);
    return transformation;
}

re_visionModelCreator::re_visionModelCreator(const std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > >& pointclouds) : pcls(pointclouds)
{
}

void re_visionModelCreator::createModel(const QDir &parent_dir, const pcl::PointCloud<pcl::PointXYZRGB> &merged_pcl, const QString& modelname) {
    QString drawing_model_filename = parent_dir.absoluteFilePath("drawing_model.ply");
    saveAsPLY(merged_pcl.makeShared(), drawing_model_filename);

    int face_counter=0;
    BOOST_FOREACH(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& pcl_ptr, pcls) {
        const pcl::PointCloud<pcl::PointXYZRGB>& pcl = *pcl_ptr;

        const QString base_filename = parent_dir.absoluteFilePath(QString("face_%1").arg(face_counter++, 3, 10, QChar('0')));

        // create BGR image from dense pointcloud
        cv::Mat3b img_color(cv::Size(pcl.width, pcl.height));
        cv::Mat1b mask(img_color.size());
        cv::Mat3b masked_img_color(img_color.size());
        size_t i=0;
        for (size_t u = 0; u < pcl.height; ++u)
        {
            for (size_t v = 0; v < pcl.width; ++v, ++i)
            {
                int32_t rgb = *(int32_t*)(&pcl.points[i].rgb);
                char b = rgb & 0xFF;
                char g = (rgb & (0xFF << 8)) >> 8;
                char r = (rgb & (0xFF << 16)) >> 16;
                img_color(u, v) = cv::Vec3b(b, g, r);
                // If the point is invalid (i.e. belongs to background)
                if (isnan (pcl.points[i].x) || isnan (pcl.points[i].y) || isnan (pcl.points[i].z))
                    mask(u, v) = 0;
		else
                    mask(u, v) = 1;
            }
        }
        std::cerr << "writing " << base_filename.toStdString() << ".png";
        cv::imwrite((base_filename+".png").toStdString(), img_color);

        cv::floodFill(mask, cv::Point2i(0, 0), 2);
        for (int u = 0; u < mask.size().height; ++u)
        {
            for (int v = 0; v < mask.size().width; ++v)
            {
                cv::Vec3b color;
                if (mask(u, v) == 2) {
                    color = cv::Vec3b(0, 0, 0);
                    mask(u, v) = 0;
                }
                else {
                    color = img_color.at<cv::Vec3b>(u, v);
                    mask(u, v) = 1;
                }

                masked_img_color.at<cv::Vec3b>(u, v) = color;
            }
        }
        cv::Mat1b masked_img_gray(masked_img_color.size());
        cv::cvtColor(masked_img_color, masked_img_gray, CV_BGR2GRAY);

        cv::GaussianBlur(masked_img_gray, masked_img_gray,  cv::Size(3,3), 0);

        DVision::SurfSet surf;
        const float surf_th = 400.;
        surf.Extract(masked_img_gray, surf_th, false);
        cout << "got " << surf.keys.size() << " descriptors" << endl;
        const int margin = 3;
        cv::Mat erosion = cv::Mat::ones(margin*2+1, margin*2+1, CV_8U);
        cv::erode(mask, mask, erosion);

        cv::Mat1b masked_eroded(masked_img_gray.size());
        for (int u = 0; u < mask.size().height; ++u)
        {
            for (int v = 0; v < mask.size().width; ++v)
            {
                uchar color;
                if (mask(u, v) == 0)
                    color = 0;
                else
                    color = masked_img_gray.at<uchar>(u, v);

                masked_eroded.at<uchar>(u, v) = color;
            }
        }

        cv::imwrite((base_filename + "_masked.png").toStdString(), masked_eroded);

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > masked_ply
		(new pcl::PointCloud<pcl::PointXYZRGB>());

        DVision::SurfSet surf_cleaned;
        for (size_t i=0; i<surf.keys.size(); i++) {
            const cv::KeyPoint& kp = surf.keys.at(i);
            const pcl::PointXYZRGB& pt3d = pcl.at(kp.pt.x, kp.pt.y);
            if ((mask(kp.pt) != 0) && (!isnan(pt3d.x)))
            {
                surf_cleaned.keys.push_back(kp);
                for (int j=0; j<surf.GetDescriptorLength(); j++) {
                    surf_cleaned.descriptors.push_back(surf.descriptors.at(i*surf.GetDescriptorLength()+j));

                }
                surf_cleaned.laplacians.push_back(surf.laplacians.at(i));
		masked_ply->push_back(pt3d);
            }
        }
        cout << "surf descriptors after removing borders: " << surf_cleaned.size() << endl;

        surf_cleaned.Save((base_filename + ".key.gz").toStdString());


        // TODO: take camera parameters from calibration, see http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html
        //Each camera entry <cameraI> contains the estimated camera intrinsics and extrinsics, and has the form:
            //<f> <k1> <k2>   [the focal length, followed by two radial distortion coeffs]
            //<R>             [a 3x3 matrix representing the camera rotation]
            //<t>             [a 3-vector describing the camera translation]
        DVision::Bundle::CameraFile::Camera cam;
        // focal length
        cam.f = 525.f;
        cam.k1 = 0.f;
        cam.k2 = 0.f;
        Eigen::Vector4f trans(pcl.sensor_origin_);
        Eigen::Quaternionf quat(pcl.sensor_orientation_);

        cv::Mat1d camera_translation(3,1);
        for (int i=0; i<3; i++) {
            camera_translation(i) = trans(i);
        }
        cam.t = camera_translation;
        cout << camera_translation << endl;

        cv::Mat1d camera_rotation(3,3);
        Eigen::Matrix3f rot_mat(quat);
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                camera_rotation(i, j) = rot_mat(i,j);
        cam.R = camera_rotation;
        cout << camera_rotation << endl;

        cam.save((base_filename + ".txt").toStdString(), "Generated by re_object_recorder");

	saveAsPLY(masked_ply, base_filename + ".ply");
    }

    createMetaFile(parent_dir, modelname, "3D", pcls.size(), 1);
}

void re_visionModelCreator::saveAsPLY(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcl, const QString &filename) {
    using DVision::PMVS::PLYFile;

#ifdef ESTIMATE_NORMALS
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    cerr << "estimating normals" << endl;

    pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(flann);
    norm_est.setInputCloud(pcl);
    norm_est.setRadiusSearch(0.01);
    norm_est.compute(*normals_cloud);
#endif

    pcl::PointCloud<pcl::PointXYZRGB> transformed_pcl;
    pcl::transformPointCloud(*pcl, transformed_pcl, getRe_visionTransformation());

    std::vector<PLYFile::PLYPoint> plyPoints;
    for(size_t  i=0; i<transformed_pcl.size(); i++) {
        const pcl::PointXYZRGB& pclPt = transformed_pcl.points.at(i);
        // nan -> point was filtered out
        if (isnan(pclPt.x))
            continue;

        PLYFile::PLYPoint plyPt;
        plyPt.x = pclPt.x;
        plyPt.y = pclPt.y;
        plyPt.z = pclPt.z;
#ifdef ESTIMATE_NORMALS
        const pcl::Normal& pclNormal = normals_cloud->points.at(i);
        plyPt.nx = pclNormal.normal_x;
        plyPt.ny = pclNormal.normal_y;
        plyPt.nz = pclNormal.normal_z;
#else
        plyPt.nx = 0;
        plyPt.ny = 0;
        plyPt.nz = 0;
#endif
        int32_t rgb = *(int32_t*)(&pclPt.rgb);
        plyPt.b = rgb & 0xFF;
        plyPt.g = (rgb & (0xFF << 8)) >> 8;
        plyPt.r = (rgb & (0xFF << 16)) >> 16;
        plyPoints.push_back(plyPt);
    }
    PLYFile::saveFile(filename.toStdString(), plyPoints);
}
