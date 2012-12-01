/** \file extractSurf.cpp
 * \brief Extracts Surf points and associates them to real point indices
 *
 * Standalone application for extracting Surf points and associating them to
 * real point indices. This also generates .key.gz and the _2d3d.txt to create
 * the final PLY files with 3D points.
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
typedef DVision::SurfSet SurfSet;

using namespace std;

void treatDirectory(const std::string &img_dir, const float fast_th);

void removeSurfPoints(SurfSet &surf, const vector<unsigned int> &i_remove);

void findClosestPoints(vector<cv::KeyPoint> &keys, 
  const vector<PixelPoint> &pixelpoints, 
  vector<int> &indices_keys,
  vector<int> &indices_pp);

void getPoints(const SurfSet &surf, const vector<PixelPoint> &pixelpoints, 
  const vector<int> &indices_keys, const vector<int> &indices_pp, 
  SurfSet &final_surf, vector<PixelPoint> &final_pixelpoints);

void saveGlobalIndices(const std::string &filename, 
  const vector<int> &indices2d, 
  const vector<PixelPoint> &pixelpoints);

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{

  if(argc < 2)
  {
    cout << "Usage: " << argv[0] << " <img dir> [SURF th = 400]"
      << endl;
    return 1;
  }

  string img_dir = argv[1];
  float surf_th = 400.f;
  if(argc >= 3) surf_th = atof(argv[2]);
  
  try
  {
    treatDirectory(img_dir, surf_th);
  }
  catch(std::string ex)
  {
    cout << ex << endl;
  }

  return 0;

}

// ---------------------------------------------------------------------------- 

void treatDirectory(const std::string &img_dir, const float surf_th)
{
  const int MARGIN = 5;
  
  vector<string> img_files = 
    DUtils::FileFunctions::Dir(img_dir.c_str(), ".jpg", true);
  
  cout << "-- " << img_files.size() << " images read" << endl; 
  
  for(unsigned int i = 0; i < img_files.size(); ++i)
  {
    const string& img_file = img_files[i];
    
    cout << img_file << "..." << endl;
    
    string path, name, ext;
    DUtils::FileFunctions::FileParts(img_file, path, name, ext);
    
    const string pp_file = path + "/" + name + "_points.txt";
    const string mask_file = path + "/" + name + "_mask.png";
    
    vector<PixelPoint> pixelpoints;
    DVision::PixelPointFile::readFile(pp_file, pixelpoints);
    
    cout << "-- " << pixelpoints.size() << " PixelPoints read from "
      << pp_file << endl;
    
    cv::Mat im = cv::imread(img_file, 0);
    
    if(im.empty())
    {
      throw string("Could not open image ") + img_file;
    }
    
    DVision::SurfSet surf;
    surf.Extract(im, surf_th, false);
    
    cout << "-- " << surf.size() << " SURF keypoints extracted" << endl;
    
    vector<unsigned int> i_out;
    
    Mask mask(mask_file.c_str());
    
    if(mask.empty())
    {
      throw string("Could not open mask ") + mask_file;
    }
    
    mask.shrink(MARGIN);
    mask.getPointsOutsideMask(surf.keys, i_out);
    
    removeSurfPoints(surf, i_out);

    // associate each fast keypoint with a 3d point
    vector<int> indices_pp; // indices of the pixelpoint vector
    vector<int> indices_keys;
    
    findClosestPoints(surf.keys, pixelpoints, indices_keys, indices_pp);
    
    // create final surfset and pixel points
    DVision::SurfSet final_surf;
    vector<PixelPoint> final_pixelpoints;
    
    getPoints(surf, pixelpoints, indices_keys, indices_pp, final_surf,
      final_pixelpoints);
    
    cout << "-- " << final_pixelpoints.size() << " final PixelPoints "
      "associated to SURF features" << endl;
    
    // save surviving surf
    const string key_file = path + "/" + name + ".key.gz";
    final_surf.Save(key_file);
    
    // save surviving plypoints
    const string map_file = path + "/" + name + "_2d3d.txt";
    // global indices of 3d features
    saveGlobalIndices(map_file, indices_pp, pixelpoints);
  }

}

// ----------------------------------------------------------------------------

void removeSurfPoints(SurfSet &surf, const vector<unsigned int> &i_remove)
{
  const int L = surf.GetDescriptorLength();
  
  vector<unsigned int> i_remove_desc;
  i_remove_desc.reserve(i_remove.size() * L);
  
  for(unsigned int i = 0; i < i_remove.size(); ++i)
  {
    int idx = i_remove[i];
    
    for(int j = 0; j < L; ++j)
    {
      i_remove_desc.push_back(idx * L + j);
    }
  }
  
  DUtils::STL::removeIndices(surf.keys, i_remove, true);
  DUtils::STL::removeIndices(surf.laplacians, i_remove, true);
  DUtils::STL::removeIndices(surf.descriptors, i_remove_desc, true);
}

// ----------------------------------------------------------------------------

void findClosestPoints(vector<cv::KeyPoint> &keys, 
  const vector<PixelPoint> &pixelpoints, 
  vector<int> &indices_keys,
  vector<int> &indices_pp)
{
  const double MAX_DIST = 2.; // px
  
  indices_keys.clear();
  indices_pp.clear();
  
  for(unsigned int i_k = 0; i_k < keys.size(); ++i_k)
  {
    const cv::KeyPoint &k = keys[i_k];
    
    double best_d = 1e9;
    int best_pp = -1;
    
    for(unsigned int i_pp = 0; i_pp < pixelpoints.size(); ++i_pp)
    {
      const PixelPoint &p = pixelpoints[i_pp];
      
      double sqd = (k.pt.x - p.u)*(k.pt.x - p.u) + (k.pt.y - p.v)*(k.pt.y - p.v);
      
      if(sqd < best_d)
      {
        best_d = sqd;
        best_pp = i_pp;
      }
    }
    
    if(best_pp > -1 && sqrt(best_d) <= MAX_DIST)
    {
      // check this points was not already set
      if(indices_pp.end() == find(indices_pp.begin(), indices_pp.end(), best_pp))
      {
        indices_pp.push_back(best_pp);
        indices_keys.push_back(i_k);
      }
    }
  }
  
}

// ----------------------------------------------------------------------------

void getPoints(const SurfSet &surf, const vector<PixelPoint> &pixelpoints, 
  const vector<int> &indices_keys, const vector<int> &indices_pp, 
  SurfSet &final_surf, vector<PixelPoint> &final_pixelpoints)
{
  final_surf.keys.clear();
  final_surf.laplacians.clear();
  final_surf.descriptors.clear();
  final_pixelpoints.clear();
  
  const int L = surf.GetDescriptorLength();
  
  vector<int>::const_iterator it;
  
  for(it = indices_keys.begin(); it != indices_keys.end(); ++it)
  {
    int idx = *it;
    final_surf.keys.push_back( surf.keys[idx] );
    final_surf.laplacians.push_back( surf.laplacians[idx] );
    
    for(int j = 0; j < L; ++j)
    {
      final_surf.descriptors.push_back( surf.descriptors[idx * L + j] );
    }
  }
  
  for(it = indices_pp.begin(); it != indices_pp.end(); ++it)
  {
    int idx = *it;
    final_pixelpoints.push_back(pixelpoints[idx]);
  }
  
}

// ----------------------------------------------------------------------------

void saveGlobalIndices(const std::string &filename, 
  const vector<int> &indices2d, 
  const vector<PixelPoint> &pixelpoints)
{
  fstream f(filename.c_str(), ios::out);
  
  f << indices2d.size() << endl;
  
  vector<int>::const_iterator iit;
  for(iit = indices2d.begin(); iit != indices2d.end(); ++iit)
  {
    int i3d = pixelpoints[*iit].idx;
    f << (iit - indices2d.begin()) << " " << i3d << endl;
  }
  
  f.close();
}

// ----------------------------------------------------------------------------

