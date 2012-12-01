/** \file createPlyPoints.cpp
 * \brief Gets a _2d3d.txt file and creates the final ply_file
 *
 * Standalone application for creating a PLY file with 3D points from a _2d3d
 * file
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

typedef DVision::PixelPointFile PixelPointFile;
typedef DVision::PixelPointFile::PixelPoint PixelPoint;
typedef DVision::SurfSet SurfSet;
typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;

using namespace std;

void treatDirectory(const std::string &img_dir);

void readPLYindices(const std::string &indice_file, 
  vector<int> &ply_indices);

void getPoints(const vector<PLYPoint> &all_plypoints, 
  const vector<int> &ply_indices, vector<PLYPoint> &plypoints);

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    cout << "Usage: " << argv[0] << " <img dir>"
      << endl;
    return 1;
  }

  string img_dir = argv[1];
  
  try
  {
    treatDirectory(img_dir);
  }
  catch(std::string ex)
  {
    cout << ex << endl;
  }

  return 0;

}

// ----------------------------------------------------------------------------

void treatDirectory(const std::string &img_dir)
{
  vector<string> img_files = 
    DUtils::FileFunctions::Dir(img_dir.c_str(), ".jpg", true);
 
  cout << "-- " << img_files.size() << " images read" << endl; 
 
  for(unsigned int i = 0; i < img_files.size(); ++i)
  {
    const string& img_file = img_files[i];
    
    cout << img_file << "..." << endl;
    
    string path, name, ext;
    DUtils::FileFunctions::FileParts(img_file, path, name, ext);
    
    const string indice_file = path + "/" + name + "_2d3d.txt";
    const string all_ply_file = path + "/" + "dense_model" + ".ply";
    
    vector<PLYPoint> all_plypoints;
    PLYFile::readFile(all_ply_file, all_plypoints);
    
    cout << "-- " << all_plypoints.size() << " PLY points read from "
      << all_ply_file << endl;
    
    vector<int> ply_indices;
    readPLYindices(indice_file, ply_indices);
    
    vector<PLYPoint> plypoints;
    getPoints(all_plypoints, ply_indices, plypoints);
    
    // save
    const string ply_file = path + "/" + name + ".ply";
    PLYFile::saveFile(ply_file, plypoints);
  }

}

// ----------------------------------------------------------------------------

void readPLYindices(const std::string &indice_file, 
  vector<int> &ply_indices)
{
  ply_indices.clear();
  
  fstream f(indice_file.c_str(), ios::in);
  
  if(!f.is_open())
  {
    throw string("Could not open file ") + indice_file;
  }
  
  int N;
  f >> N;
  
  for(int i = 0; i < N; ++i)
  {
    int key_idx, ply_idx;
    f >> key_idx >> ply_idx;
    
    ply_indices.push_back(ply_idx);
  }
  
  f.close();
}

// ----------------------------------------------------------------------------

void getPoints(const vector<PLYPoint> &all_plypoints, 
  const vector<int> &ply_indices, vector<PLYPoint> &plypoints)
{
  plypoints.clear();
  plypoints.reserve(ply_indices.size());
  
  vector<int>::const_iterator it;
  for(it = ply_indices.begin(); it != ply_indices.end(); ++it)
  {
    int idx = *it;
    plypoints.push_back(all_plypoints[idx]);
  }
}

// ----------------------------------------------------------------------------

