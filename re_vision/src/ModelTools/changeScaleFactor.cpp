/** \file changeScaleFactor.cpp
 * \brief Modifies the scale factor of a point cloud 3D model (type "3D")
 *
 * Changes the scale factor of a point cloud 3D model (type "3D") by modifying
 * all the ply files and the scale factor of the meta.xml file.
 * The scale factor in the meta file is updated so that the final value is
 * the scale factor to apply to the output of Bundler+PMVS in order to build
 * the same model again.
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
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

/**
 * Usage: changeScaleFactor <model dir> <relative scale factor>
 *
 * model dir: directory where the model is stored
 * relative scale factor: the scale factor to apply to the current scale
 *    factor of the model
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
 * Multiplies the coordinates of all the given points by s
 * @param plypoints
 * @param s
 */
void updatePointCloud(std::vector<PLYPoint> &plypoints, float s);

// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "changeScaleFactor");
 
  if(argc < 3)
  {
    ROS_WARN("Usage: %s <model dir> <relative scale factor>", argv[0]);
    return 1;
  }
  
  string model_dir = argv[1];
  float relative_scale_factor = atof(argv[2]);
  
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
  
  if(meta.Type != "3D")
  {
    ROS_WARN("Object %s is of type %s. Only \"3D\" models are supported",
      meta.Name.c_str(), meta.Type.c_str());
    return 3;
  }
  
  float current_scale_factor = meta.Dimensions.Volume.Scale;
  float final_scale_factor = current_scale_factor * relative_scale_factor;
  
  ROS_INFO("Changing scale factor of object %s from %f to %f (%f x %f = %f)...",
    meta.Name.c_str(), current_scale_factor, final_scale_factor,
    current_scale_factor, relative_scale_factor, final_scale_factor);
  
  // dense model  
  vector<PLYPoint> plypoints;
  
  string drawing_model_file = model_dir + "/drawing_model.ply";
  PLYFile::readFile(drawing_model_file, plypoints);  
  updatePointCloud(plypoints, relative_scale_factor);
  PLYFile::saveFile(drawing_model_file, plypoints);

  // face points
  for(int i = 0; i < meta.NFaces; ++i)
  {
    stringstream ply_file;
    ply_file << model_dir << "/face_" << setw(3) << setfill('0') << i
      << ".ply";
    
    PLYFile::readFile(ply_file.str(), plypoints);  
    updatePointCloud(plypoints, relative_scale_factor);
    PLYFile::saveFile(ply_file.str(), plypoints);
  }
  
  // meta file
  meta.Dimensions.Volume.Scale = final_scale_factor;
  MetaFile::saveFile(meta_file, meta);
  
  // done
  ROS_INFO("Done. Drawing model, %d faces and meta file modified", meta.NFaces);
  
  return 0;
}
	
// ---------------------------------------------------------------------------

void updatePointCloud(std::vector<PLYPoint> &plypoints, float s)
{
  vector<PLYPoint>::iterator pit;
  for(pit = plypoints.begin(); pit != plypoints.end(); ++pit)
  {
    pit->x *= s;
    pit->y *= s;
    pit->z *= s;
  }
}

// ---------------------------------------------------------------------------

