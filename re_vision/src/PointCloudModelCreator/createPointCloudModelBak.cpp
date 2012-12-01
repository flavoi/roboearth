// THIS FILE IS NOT USED YET

/** \file createPointCloudModel.cpp
 * \brief Creates a 3D point cloud object model
 *
 * Creates a point cloud object model from a set of images
 * 
 * This file is part of the RoboEarth ROS WP1 package.
 * 
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2010 by <a href="mailto:dorian@unizar.es">Dorian Galvez-Lopez</a>, University of Zaragoza
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \author Dorian Galvez-Lopez
 * \version 1.0
 * \date 2010
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

/**
 * Usage: createPlanarModel <training image> <object name> <width (m)> 
 *        <height (m)> <output model dir> [surf threshold]
 *
 * training image: undistorted image of the object
 * object name
 * width, height: real width and height of the object in metres
 * output model dir: directory to store the files of the model in (must exist)
 *
 */

#include "ros/ros.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../MetaFile.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;
typedef DVision::Bundle::CameraFile::Camera Camera;

using namespace std;

// ---------------------------------------------------------------------------

/**
 * Generates the 3D points of the given surf features. The 3D points are given
 * in the object reference, located in the center of the object, x pointing right,
 * y pointing down, z going away the camera.
 * @param img source color image
 * @param width
 * @param height dimensions of the real object in metres. One dimension can be
 *   zero
 * @param surfset surfs extracted from img
 * @param plypoint created 3d points
 */
void createPLYpoints(const cv::Mat &img, float width, float height,
  const DVision::SurfSet &surfset, std::vector<PLYPoint> &plypoints);

/**
 * Creates a virtual camera that would watch the object in front of it
 * The virtual camera is suposed to be oriented as the object, at a distance
 * of 1 metre
 * @param im image
 * @param width
 * @param height real dimensions of the given image
 * @param camera virtual camera
 */
void createCamera(const cv::Mat &im, float width, float height,
  Camera &camera);

/**
 * Generates under the directory model_dir the files of one face of the model
 * @param model_dir output model directory
 * @param face_idx index of this face
 * @param img face image
 * @param surfs surfs from img
 * @param plypoints 3d coords of the surfs keypoints
 * @param camera intrinsic parameters of camera
 */
void saveFace(const std::string &model_dir, int face_idx, const cv::Mat &img,
  const DVision::SurfSet &surfs, const std::vector<PLYPoint> &plypoints,
  const Camera &camera);

// ---------------------------------------------------------------------------

static const int DEFAULT_HESSIAN_THRESHOLD = 400;

// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "createPlanarModel");
 
  if(argc < 6)
  {
    ROS_WARN("Usage: createPlanarModel <training image> <object name> "
      "<width (m)> <height (m)> <output model dir> [surf threshold]");
    return 1;
  }
  
  cout << argc << endl;
  
  string image_file = argv[1];
  string object_name = argv[2];
  float width = atof(argv[3]);
  float height = atof(argv[4]);
  string out_model_dir = argv[5];
  int hessian = (argc >= 7 ? atoi(argv[6]) : DEFAULT_HESSIAN_THRESHOLD);
  
  ROS_INFO("Reading image...");
  cv::Mat img_color = cv::imread(image_file, 1);
  cv::Mat bw_img = cv::imread(image_file, 0);
  
  ROS_INFO("Extracting SURFs (hessian threshold: %d)...", hessian);
  DVision::SurfSet surfset;
  surfset.Extract(bw_img, hessian);
  ROS_INFO("  %d points obtained", surfset.size());
  
  ROS_INFO("Computing 3D coords and camera info...");
  vector<PLYPoint> plypoints;
  createPLYpoints(img_color, width, height, surfset, plypoints);
  
  Camera camera;
  createCamera(img_color, width, height, camera);
  
  ROS_INFO("Creating meta data...");
  MetaFile::MetaData meta;
  meta.Name = object_name;
  meta.NFaces = 1;
  meta.Type = "planar";
  meta.Dimensions.Planar.Width = width;
  meta.Dimensions.Planar.Height = height;
  meta.Dimensions.Planar.Depth = 0;
  meta.Dimensions.Planar.Faces.resize(1);
  meta.Dimensions.Planar.Faces[0].Width = width;
  meta.Dimensions.Planar.Faces[0].Height = height;
  meta.Dimensions.Planar.Faces[0].oTf = cv::Mat::eye(4,4,CV_64F);
  
  // not necessary anylonger
  //ROS_INFO("Building visualization model...");
  //cv::imwrite(out_model_dir + "/drawing_model.png", img_color);
  
  ROS_INFO("Saving...");
  
  if(!DUtils::FileFunctions::DirExists(out_model_dir.c_str()))
    DUtils::FileFunctions::MkDir(out_model_dir.c_str());
  
  saveFace(out_model_dir, 0, img_color, surfset, plypoints, camera);
  MetaFile::saveFile(out_model_dir + "/meta.xml", meta);
  
  ROS_INFO("Done");
  
  return 0;
}
	
// ---------------------------------------------------------------------------

void createPLYpoints(const cv::Mat &img, float width, float height,
  const DVision::SurfSet &surfset, std::vector<PLYPoint> &plypoints)
{
  plypoints.resize(0);
  if(surfset.keys.empty()) return;
  
  double mperpixel;
  if(width > 0.f && height > 0.f)
    mperpixel = ((width / (float)img.cols) + (height / (float)img.rows)) / 2.f;
  else if(width > 0.f) mperpixel = width / (float)img.cols;
  else mperpixel = height / (float)img.rows;
  
  const float cx = (float)img.cols / 2.f;
  const float cy = (float)img.rows / 2.f;
  
  vector<cv::Mat> bgr_planes;
  cv::split(img, bgr_planes);
  
  plypoints.reserve(surfset.size());
  
  vector<cv::KeyPoint>::const_iterator kit;
  for(kit = surfset.keys.begin(); kit != surfset.keys.end(); ++kit)
  {
    plypoints.push_back(PLYPoint());
    PLYPoint &p = plypoints.back();
 
    p.x = (kit->pt.x - cx) * mperpixel;
    p.y = (kit->pt.y - cy) * mperpixel;
    p.z = 0.;
    
    p.nx = 0.;
    p.ny = 0.;
    p.nz = 1.;

    p.r = bgr_planes[2].at<uchar>(kit->pt.y, kit->pt.x);
    p.g = bgr_planes[1].at<uchar>(kit->pt.y, kit->pt.x);
    p.b = bgr_planes[0].at<uchar>(kit->pt.y, kit->pt.x);
  }
  
}

// ---------------------------------------------------------------------------

void createCamera(const cv::Mat &im, float width, float height,
  Camera &camera)
{
  cv::Mat cTo = DUtilsCV::Transformations::transl(0, 0, 1);
  
  float fx = im.cols / width;
  float fy = im.rows / height;
  camera.f = (fx + fy) / 2;
  camera.k1 = camera.k2 = 0;
  
  camera.t = cv::Mat::zeros(3, 1, CV_64F);
  DUtilsCV::Transformations::decomposeRt(cTo, camera.R, camera.t);
}

// ---------------------------------------------------------------------------

void saveFace(const std::string &model_dir, int face_idx, const cv::Mat &img,
  const DVision::SurfSet &surfs, const std::vector<PLYPoint> &plypoints,
  const Camera &camera)
{
  stringstream prefix;
  prefix << model_dir << "/face_" << setw(3) << setfill('0') << face_idx;
  
  cv::imwrite(prefix.str() + ".png", img);
  surfs.Save(prefix.str() + ".key.gz");
  PLYFile::saveFile(prefix.str() + ".ply", plypoints);
  camera.save(prefix.str() + ".txt", 
    "Fake camera. Assumes cx=w/2, cy=h/2, Z_object=1");
}

// ---------------------------------------------------------------------------

