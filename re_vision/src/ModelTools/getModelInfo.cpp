/** \file getModelInfo.cpp
 * \brief Retrieves information from models
 *
 * Reads model files and prints its information in the screen
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
 * Usage: getModelInfo <model dir>
 *
 * model dir: directory where the model is stored
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
#include "../ObjectModel.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;
typedef DVision::SurfSet SurfSet;

using namespace std;

// ---------------------------------------------------------------------------

/**
 * Returns the minimum value of the hessian of the given keypoints
 * @param surf keypoints
 * @return min hessian
 */
double getMinHessian(const SurfSet &surf);

/**
 * Calculates the dimensions in the object axes
 * @param plypoints dense point cloud of the model
 * @param dx
 * @param dy
 * @param dz (out) dimensions
 */
void calculatePointCloudDimensions(const std::vector<PLYPoint> &plypoints, 
  float &dx, float &dy, float &dz);

// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getModelInfo");
 
  if(argc < 2)
  {
    ROS_WARN("Usage: %s <model dir>", argv[0]);
    return 1;
  }
  
  string model_dir = argv[1];
  
  if(!ObjectModel::checkDirectory(model_dir))
  {
    ROS_ERROR("Directory %s does not seem to contain a valid object model",
      model_dir.c_str());
    return 2;
  }
  
  string meta_file = model_dir + "/meta.xml";
  ROS_DEBUG("Reading meta file %s...", meta_file.c_str());
  MetaFile::MetaData meta;
  MetaFile::readFile(meta_file, meta);
  
  ROS_INFO("Retrieving object info from %s...", model_dir.c_str());
  ROS_INFO("Object name: %s", meta.Name.c_str());
  ROS_INFO("Type: %s", meta.Type.c_str());
  
  if(meta.Type == "planar")
  {
    ROS_INFO("Dimensions (width x height x depth, metres): %f x %f x %f", 
      meta.Dimensions.Planar.Width, meta.Dimensions.Planar.Height,
      meta.Dimensions.Planar.Depth);
  }
  else if(meta.Type == "3D")
  {
    ROS_INFO("Scale factor: %f", meta.Dimensions.Volume.Scale);
    
    string drawing_model_file = model_dir + "/drawing_model.ply";
    vector<PLYPoint> drawing_plypoints;
    PLYFile::readFile(drawing_model_file, drawing_plypoints);
    
    float dx, dy, dz;
    calculatePointCloudDimensions(drawing_plypoints, dx, dy, dz);
    
    ROS_INFO("Dimensions (X, Y, Z axes): %f x %f x %f", dx, dy, dz);
  }
  else
  {
    ROS_INFO("(Unknown dimensions)");
  }

  ROS_INFO("Faces: %d", meta.NFaces);
  
  // get info from each face
  for(int i = 0; i < meta.NFaces; ++i)
  {
    //stringstream ply_file;
    //ply_file << model_dir << "/face_" << setw(3) << setfill('0') << i
    //  << ".ply";
    stringstream surf_file;
    surf_file << model_dir << "/face_" << setw(3) << setfill('0') << i
      << ".key.gz";
    
    DVision::SurfSet surf;
    surf.Load(surf_file.str());
    double min_hessian = getMinHessian(surf);
    
    if(meta.Type == "planar")
    {
      ROS_INFO("- Face %d: %d points (min hessian: %f), %f x %f metres", i, 
        surf.size(), min_hessian,
        //PLYFile::getNumberOfPoints(ply_file.str()),
        meta.Dimensions.Planar.Faces[i].Width,
        meta.Dimensions.Planar.Faces[i].Height);
    }
    else
    {
      ROS_INFO("- Face %d: %d points (min hessian: %f)", i, 
        surf.size(), min_hessian);
        //PLYFile::getNumberOfPoints(ply_file.str()));
    }
    
  }
  
  return 0;
}
	
// ---------------------------------------------------------------------------

double getMinHessian(const SurfSet &surf)
{
  if(surf.keys.empty()) return 0;
  
  double min = 1e9;
  vector<cv::KeyPoint>::const_iterator kit;
  for(kit = surf.keys.begin(); kit != surf.keys.end(); ++kit)
  {
    if(kit->response < min) min = kit->response;
  }
  return min;
}

// ---------------------------------------------------------------------------

void calculatePointCloudDimensions(const std::vector<PLYPoint> &plypoints, 
  float &dx, float &dy, float &dz)
{
  float xmin, xmax, ymin, ymax, zmin, zmax;
  
  if(plypoints.empty())
  {
    dx = dy = dz = 0;
  }
  else
  {
    vector<PLYPoint>::const_iterator it = plypoints.begin();
    xmin = xmax = it->x;
    ymin = ymax = it->y;
    zmin = zmax = it->z;
    
    for(++it; it != plypoints.end(); ++it)
    {
      if(it->x < xmin) xmin = it->x;
      else if(it->x > xmax) xmax = it->x;

      if(it->y < ymin) ymin = it->y;
      else if(it->y > ymax) ymax = it->y;
      
      if(it->z < zmin) zmin = it->z;
      else if(it->z > zmax) zmax = it->z;
    }
    
    dx = xmax - xmin;
    dy = ymax - ymin;
    dz = zmax - zmin;
  }
}

// ---------------------------------------------------------------------------

