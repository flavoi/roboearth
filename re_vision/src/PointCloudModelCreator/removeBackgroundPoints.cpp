/** \file removeBackgroundPoints.cpp
 * \brief Removes points that lie outside the masks 
 *
 * Standalone application for removing feature points that are not inside
 * the mask of the object
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
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <vector>

#include "Mask.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

typedef DVision::PixelPointFile::PixelPoint PixelPoint;
typedef DVision::PMVS::PatchFile PatchFile;
typedef DVision::PMVS::PatchFile::Patch Patch;
typedef DVision::PMVS::CameraFile CameraFile;
typedef DVision::PMVS::CameraFile::Camera Camera;
typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;

using namespace std;

// ----------------------------------------------------------------------------

// old
void treatAll(const std::vector<PLYPoint> &allplypoints, 
  const std::vector<Camera> &cameras, 
  const std::vector<std::vector<int> > &visibility, 
  const std::vector<Mask> &masks,
  std::vector<PLYPoint> &object_plypoints);

void loadMasks(const std::string &mask_dir, int N,
  std::vector<Mask> &masks);

void treatAll(const std::vector<PLYPoint> &allplypoints, 
  const std::vector<Camera> &cameras, 
  const std::vector<Patch> &patches, 
  const std::vector<Mask> &masks,
  std::vector<PLYPoint> &object_plypoints,
  std::vector<Patch> &object_patches);

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{

  if(argc < 7)
  {
    cout << "Usage: " << argv[0] << " <complete ply file> <patch file> "
      "<camera dir> <mask dir> <out ply file> <out patch file>" 
      << endl;
    return 1;
  }

  string ply_in_file = argv[1];
  string patch_file = argv[2];
  string camera_dir = argv[3];
  string mask_dir = argv[4];
  string ply_out_file = argv[5];
  string patch_out_file = argv[6];

  try 
  {

    cout << "Reading data..." << endl;
    
    vector<PLYPoint> allplypoints;
    PLYFile::readFile(ply_in_file, allplypoints);
    
    cout << "-- " << allplypoints.size() << " PLY points read" << endl;
    
    vector<Camera> cameras;
    CameraFile::readFile(camera_dir, cameras);
    
    cout << "-- " << cameras.size() << " cameras read" << endl;
    
    //vector<vector<int> > visibility;
    //PatchFile::readFile(patch_file, visibility);
    vector<Patch> patches;
    PatchFile::readFile(patch_file, patches);

    cout << "-- " << patches.size() << " patches read" << endl;

    cout << "Selecting points..." << endl;

    vector<Mask> masks;
    loadMasks(mask_dir, cameras.size(), masks);
    
    cout << "-- " << masks.size() << " masks read" << endl;

    vector<PLYPoint> object_plypoints;
    vector<Patch> object_patches;
    
    //treatAll(allplypoints, cameras, visibility, masks, object_plypoints);
    treatAll(allplypoints, cameras, patches, masks, object_plypoints,
      object_patches);
    
    cout << "Saving..." << endl;
    
    PLYFile::saveFile(ply_out_file, object_plypoints);
    PatchFile::saveFile(patch_out_file, object_patches);
  
  }catch(std::string ex)
  {
    cout << ex << endl;
  }

  return 0;

}

// ----------------------------------------------------------------------------

void loadMasks(const std::string &mask_dir, int N,
  std::vector<Mask> &masks)
{
  const int MARGIN = 1;
  char buffer[1024];
  masks.reserve(N);
  
  for(int i = 0; i < N; ++i)
  {
    sprintf(buffer, "%s/im%02d_mask.png", mask_dir.c_str(), i);
    masks.push_back(Mask(buffer));
    
    if(masks.back().empty())
    {
      throw string("Could not read mask file ") + buffer;
    }
    
    masks.back().shrink(MARGIN);
  }
}

// ----------------------------------------------------------------------------

// DEPRECATED
void treatAll(const std::vector<PLYPoint> &allplypoints, 
  const std::vector<Camera> &cameras, 
  const std::vector<std::vector<int> > &visibility, 
  const std::vector<Mask> &masks,
  std::vector<PLYPoint> &object_plypoints)
{
  object_plypoints.clear();
  object_plypoints.reserve(allplypoints.size());
  
  for(unsigned int i = 0; i < visibility.size(); ++i)
  {
    // visibility[i] = points seen from the i-th camera
    const Camera& camera = cameras[i];
    const std::vector<int> &visible = visibility[i];
    const Mask &mask = masks[i];

    for(unsigned int j = 0; j < visible.size(); ++j)
    {
      int idx = visible[j];
      const PLYPoint& p3d = allplypoints[idx];
    
      cv::Mat X = (cv::Mat_<double>(4, 1) << 
        p3d.x, p3d.y, p3d.z, 1.);
      
      cv::Mat p = camera.P * X;
      
      int x = int(p.at<double>(0,0) / p.at<double>(2,0));
      int y = int(p.at<double>(1,0) / p.at<double>(2,0));

      // add if it is inside the mask
      if(mask.test(x, y))
      {
        object_plypoints.push_back(p3d);
      }
    }
  }
}

// ----------------------------------------------------------------------------

void treatAll(const std::vector<PLYPoint> &allplypoints, 
  const std::vector<Camera> &cameras, 
  const std::vector<Patch> &patches, 
  const std::vector<Mask> &masks,
  std::vector<PLYPoint> &object_plypoints,
  std::vector<Patch> &object_patches)
{
  object_plypoints.clear();
  object_plypoints.reserve(allplypoints.size());
  
  object_patches.clear();
  object_patches.reserve(allplypoints.size());
  
  for(unsigned int idx = 0; idx < patches.size(); ++idx)
  {
    const Patch& patch = patches[idx];
    const PLYPoint& p3d = allplypoints[idx];
    
    bool ok = true;
    for(unsigned int j = 0; j < patch.strong_visibility_list.size(); ++j)
    {
      int img_idx = patch.strong_visibility_list[j];
      const Camera& camera = cameras[img_idx];
      const Mask& mask = masks[img_idx];
      
      cv::Mat X = (cv::Mat_<double>(4, 1) << 
        p3d.x, p3d.y, p3d.z, 1.);
      
      cv::Mat p = camera.P * X;
      
      int x = int(p.at<double>(0,0) / p.at<double>(2,0));
      int y = int(p.at<double>(1,0) / p.at<double>(2,0));

      // add if it is inside the mask
      if(!mask.test(x, y))
      {
        ok = false;
        break;
      }
    }
    
    if(ok)
    {
      // add this point
      object_plypoints.push_back(p3d);
      object_patches.push_back(patch);
    }
  }
  
}

// ----------------------------------------------------------------------------

