/** \file computePixelPoints.cpp
 * \brief Gets visible 2D points from a 3D point cloud
 *
 * Standalone application for computing the visible 2D points from a 3D 
 * point cloud
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
#include <vector>

#include "Mask.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

using namespace std;

typedef DVision::PMVS::PatchFile PatchFile;
typedef DVision::PMVS::PatchFile::Patch Patch;

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;

typedef DVision::PMVS::CameraFile CameraFile;
typedef DVision::PMVS::CameraFile::Camera Camera;

// ----------------------------------------------------------------------------

struct PixelPoint
{
  float x, y; // coords
  int idx3d; // index of its plypoint
};

void getPixelsFromImage(const std::string &img_file, 
  const std::string &mask_file, 
  const Camera& camera,
  const std::vector<PLYPoint> &plypoints, 
  const std::vector<int> &visible_points,
  vector<PixelPoint> &pixelpoints);

void savePixelPoints(const std::string &save_file, 
  const vector<PixelPoint> &pixel_points,
  const vector<PLYPoint> &plypoints);

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
  cout << "(ply ok) " << flush;
  
  vector<vector<int> > visibility;
  PatchFile::readFile(patch_file, visibility);
  cout << "(patch ok) " << flush;

  vector<Camera> cameras;
  CameraFile::readFile(camera_dir, cameras);
  cout << "(cameras ok) " << flush;
  
  cout << endl;
  
  vector<string> mask_files =
    DUtils::FileFunctions::Dir(img_dir.c_str(), "_mask.png", true);
  
  if(mask_files.size() != cameras.size())
  {
    cout << "Error: there are " << cameras.size() << " cameras, but "
      << mask_files.size() << " images" << endl;
    return 1;
  }
  
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
  
  // save (rename) the cameras
  if(!mask_files.empty())
  {
    string path, name, ext;
    DUtils::FileFunctions::FileParts(mask_files[0], path, name, ext);    
    CameraFile::saveFile(path, cameras, "im%02d_cam.txt");
  }

  return 0;
}

// ----------------------------------------------------------------------------

void getPixelsFromImage(const std::string &img_file, 
  const std::string &mask_file, 
  const Camera &camera,
  const std::vector<PLYPoint> &plypoints, 
  const std::vector<int> &visible_points,
  vector<PixelPoint> &pixelpoints)
{
  const int MARGIN = 5;
  
  cv::Mat im = cv::imread(img_file.c_str(), 0);
  Mask mask(mask_file.c_str());
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


