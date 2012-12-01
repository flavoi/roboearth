/** \file extractFast.cpp
 * \brief Extracts FAST points from images and associates them to 
 *  real point indices
 *
 * Standalone application for extracting FAST points from images and associates 
 * them to real point indices.
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

using namespace std;

void treatDirectory(const std::string &img_dir, const float fast_th);

void findClosestPoints(vector<cv::KeyPoint> &keys, 
  const vector<PixelPoint> &pixelpoints, 
  vector<int> &indices2d);
  
void saveGlobalIndices(const std::string &filename, 
  const vector<int> &indices2d, 
  const vector<PixelPoint> &pixelpoints);

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{

  if(argc < 2)
  {
    cout << "Usage: " << argv[0] << " <img dir> [FAST th = 0]"
      << endl;
    return 1;
  }

  string img_dir = argv[1];
  float fast_th = 0.f;
  if(argc >= 3) fast_th = atof(argv[2]);
  
  treatDirectory(img_dir, fast_th);

  return 0;

}

// ---------------------------------------------------------------------------- 

void treatDirectory(const std::string &img_dir, const float fast_th)
{
  const int MARGIN = 5;
  
  vector<string> img_files = 
    DUtils::FileFunctions::Dir(img_dir.c_str(), ".jpg", true);
  
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
    
    cv::Mat im = cv::imread(img_file, 0);
    
    vector<cv::KeyPoint> keys;
    cv::FAST(im, keys, fast_th);
    
    Mask mask(mask_file.c_str());
    mask.shrink(MARGIN);
    mask.maskKeyPoints(keys);
    
    // save all the keypoints for debugging
    {
      const string key_file = path + "/" + name + "_fast_all.key";
      cv::FileStorage fs(key_file, cv::FileStorage::WRITE);
      cv::write(fs, "fast", keys);
      fs.release();
    }

    // associate each fast keypoint with a 3d point
    vector<int> indices2d; // indices of the pixelpoint vector
    findClosestPoints(keys, pixelpoints, indices2d);
    
    // save surviving fast points
    const string key_file = path + "/" + name + "_fast.key";
    cv::FileStorage fs(key_file, cv::FileStorage::WRITE);
    cv::write(fs, "fast", keys);
    fs.release();
    
    // and index map
    const string map_file = path + "/" + name + "_2d3d.txt";
    // global indices of 3d features
    saveGlobalIndices(map_file, indices2d, pixelpoints);
  }

}

// ----------------------------------------------------------------------------

void findClosestPoints(vector<cv::KeyPoint> &keys, 
  const vector<PixelPoint> &pixelpoints, 
  vector<int> &indices2d)
{
  const double MAX_DIST = 2.; // px
  
  indices2d.clear();
  
  int deleted = 0;
  unsigned int i = 0;
  while(i < keys.size() - deleted)
  {
    const cv::KeyPoint &k = keys[i];
    
    double best_d = 1e9;
    int best_j = -1;
    
    for(unsigned int j = 0; j < pixelpoints.size(); ++j)
    {
      const PixelPoint &p = pixelpoints[j];
      
      double sqd = (k.pt.x - p.u)*(k.pt.x - p.u) + (k.pt.y - p.v)*(k.pt.y - p.v);
      
      if(sqd < best_d)
      {
        best_d = sqd;
        best_j = j;
      }
    }
    
    if(best_j > -1 && sqrt(best_d) <= MAX_DIST)
    {
      // keep this point and save the best corresponding index
      indices2d.push_back(best_j);
      ++i;
    }
    else
    {
      // delete this keypoint
      keys[i] = keys[keys.size()-1 - deleted];
      deleted++;
    }
  }
  
  if(deleted > 0) keys.resize(keys.size() - deleted);
  
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

