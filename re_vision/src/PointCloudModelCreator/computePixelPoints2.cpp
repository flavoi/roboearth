/** \file computePixelPoints2.cpp
 * \brief Gets visible 2D points from a 3D point cloud, and transform
 * PMVS cameras into Bundler cameras
 *
 * Standalone application for computing the visible 2D points from a 3D 
 * point cloud and for transforming PMVS cameras into Bundler cameras.
 * 
 * This file is part of the RoboEarth ROS WP1 package.
 * 
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by <a href="mailto:dorian@unizar.es">Dorian Galvez-Lopez</a>, University of Zaragoza
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
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>

#include "Mask.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

using namespace std;

typedef DVision::PMVS::PatchFile PatchFile;
typedef DVision::PMVS::PatchFile::Patch Patch;

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;

typedef DVision::PMVS::CameraFile PMVS_CameraFile;
typedef DVision::PMVS::CameraFile::Camera PMVS_Camera;

typedef DVision::Bundle::CameraFile Bundle_CameraFile;
typedef DVision::Bundle::CameraFile::Camera Bundle_Camera;


// ----------------------------------------------------------------------------

struct PixelPoint
{
  float x, y; // coords
  int idx3d; // index of its plypoint
};

void getPixelsFromImage(const std::string &img_file, 
  const std::string &mask_file, 
  const PMVS_Camera &camera,
  const std::vector<PLYPoint> &plypoints, 
  const std::vector<int> &visible_points,
  vector<PixelPoint> &pixelpoints);

void savePixelPoints(const std::string &save_file, 
  const vector<PixelPoint> &pixel_points,
  const vector<PLYPoint> &plypoints);

void convertCamera(const PMVS_Camera &pmvs_camera, 
  Bundle_Camera &bundle_cam);

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{

  if(argc < 5)
  {
    cout << "Usage: " << argv[0] << " <ply file> <patch file> <img dir> <camera dir>"
      << endl;
    return 1;
  }

  string ply_file = argv[1];
  string patch_file = argv[2];
  string img_dir = argv[3];
  string camera_dir = argv[4];
  
  cout << "Loading input files... " << flush;
  
  vector<PLYPoint> plypoints;
  PLYFile::readFile(ply_file, plypoints);  
  cout << plypoints.size() << " PLY points read" << endl;
  
  vector<vector<int> > visibility;
  PatchFile::readFile(patch_file, visibility);
  cout << "Covisibility values read" << endl;

  vector<PMVS_Camera> cameras;
  PMVS_CameraFile::readFile(camera_dir, cameras);
  cout << cameras.size() << " cameras read" << endl;
  
  cout << endl;
  
  vector<string> mask_files =
    DUtils::FileFunctions::Dir(img_dir.c_str(), "_mask.png", true);
  
  if(mask_files.size() != cameras.size())
  {
    cout << "Error: there are " << cameras.size() << " cameras, but "
      << mask_files.size() << " images" << endl;
    return 1;
  }
  
  try
  {
  
    for(unsigned int i = 0; i < mask_files.size(); ++i)
    {
      string path, name, ext;
      DUtils::FileFunctions::FileParts(mask_files[i], path, name, ext);
      
      string img_name = name.substr(0, name.size() - string("_mask").length());
      string img_file = path + "/" + img_name + ".jpg";
      
      cout << img_file << "..." << endl;

      // projection
      vector<PixelPoint> pixelpoints;
      getPixelsFromImage(img_file, mask_files[i], cameras[i],
        plypoints, visibility[i], pixelpoints);

      // save
      string save_file = path + "/" + img_name + "_points.txt";
      savePixelPoints(save_file, pixelpoints, plypoints);
    }
    
    // save (rename) the cameras in bundler format
    if(!mask_files.empty())
    {
      cout << "Creating cameras..." << endl;
      
      string path, name, ext;
      DUtils::FileFunctions::FileParts(mask_files[0], path, name, ext);
      
      vector<Bundle_Camera> bundle_cams(1);
      for(unsigned int i = 0; i < cameras.size(); ++i)
      {
        cout << "... camera " << i << endl;
        
        convertCamera(cameras[i], bundle_cams[0]);
        
        cout << "... saving..." << endl;
        
        stringstream ssfile;
        ssfile << path << "/im" << setw(2) << setfill('0') << i << "_cam.txt";
        Bundle_CameraFile::saveFile(ssfile.str(), bundle_cams);
      }
    }
  
  }catch(std::string ex)
  {
    cout << ex << endl;
  }

  return 0;
}

// ----------------------------------------------------------------------------

void getPixelsFromImage(const std::string &img_file, 
  const std::string &mask_file, 
  const PMVS_Camera &camera,
  const std::vector<PLYPoint> &plypoints, 
  const std::vector<int> &visible_points,
  vector<PixelPoint> &pixelpoints)
{
  const int MARGIN = 5;
  
  cv::Mat im = cv::imread(img_file.c_str(), 0);
  
  if(im.empty())
  {
    throw string("Could not open image ") + img_file;
  }
  
  Mask mask(mask_file.c_str());
  
  if(mask.empty())
  {
    throw string("Could not open mask ") + mask_file;
  }
  
  mask.shrink(MARGIN);
  
  pixelpoints.resize(0);
  pixelpoints.reserve(visible_points.size());
  
  std::vector<int>::const_iterator vit;
  for(vit = visible_points.begin(); vit != visible_points.end(); ++vit)
  {
    const PLYPoint& p3d = plypoints[*vit];
    
    cv::Mat X = (cv::Mat_<double>(4, 1) << 
      p3d.x, p3d.y, p3d.z, 1.);
    
    cv::Mat p = camera.P * X;

    PixelPoint pp;
    pp.x = float(p.at<double>(0,0) / p.at<double>(2,0));
    pp.y = float(p.at<double>(1,0) / p.at<double>(2,0));
    pp.idx3d = *vit;

    // add if it is inside the mask
    if(mask.test((int)pp.x, (int)pp.y))
    {
      pixelpoints.push_back(pp);
    }
  }
  
}

// ----------------------------------------------------------------------------

void savePixelPoints(const std::string &save_file, 
  const vector<PixelPoint> &pixel_points, const vector<PLYPoint> &plypoints)
{
  // Format:
  // N
  // u v x y z idx3d
  // ...
  
  fstream f(save_file.c_str(), ios::out);
  
  f << pixel_points.size() << endl;
  
  vector<PixelPoint>::const_iterator pit;
  for(pit = pixel_points.begin(); pit != pixel_points.end(); ++pit)
  {
    f << pit->x << " "
      << pit->y << " "
      << plypoints[pit->idx3d].x << " "
      << plypoints[pit->idx3d].y << " "
      << plypoints[pit->idx3d].z << " "
      << pit->idx3d << endl;
  }

  f.close();
}

// ----------------------------------------------------------------------------

void convertCamera(const PMVS_Camera &pmvs_camera, 
  Bundle_Camera &bundle_cam)
{
  cv::Mat K, R, t; // 3x3, 3x3, 4x1
  cv::decomposeProjectionMatrix(pmvs_camera.P, K, R, t);
  // opencv 2.1: t is not correct 

  assert(K.type() == CV_64F);
  assert(R.type() == CV_64F);
  assert(t.type() == CV_64F);
  
  cv::Mat Rt = K.inv() * pmvs_camera.P;
  
  bundle_cam.f = K.at<double>(0,0);
  bundle_cam.k1 = bundle_cam.k2 = 0;
  
  bundle_cam.R = Rt.rowRange(0, 3).colRange(0, 3);
  bundle_cam.t = Rt.rowRange(0, 3).colRange(3, 4);
  
}

// ----------------------------------------------------------------------------

