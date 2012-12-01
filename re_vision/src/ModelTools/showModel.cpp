/** \file showModel.cpp
 * \brief Draws the given model 
 *
 * Reads model files and draws the model in an image
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
 * Usage: showModel <model dir> [ z v1 v2 v3 a <output img> ]
 *
 * model dir: directory where the model is stored
 * z: distance of object in metres (1 by default)
 * v1, v2, v3: rotation vector (no rotation by default)
 * a: rotation angle in degrees
 * output img: if given, the generated image is saved here
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

using namespace std;

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "showModel");
  
  if(argc < 2)
  {
    ROS_WARN("Usage: %s <model dir> [ z v1 v2 v3 a <output img> ]", argv[0]);
    return 1;
  }
  
  string model_dir = argv[1];
  string output_img = (argc >= 8 ? argv[7] : "");
  float z = (argc >= 3 ? atof(argv[2]) : 1.f);
  bool there_is_rotation = (argc >= 7);
  float r[3], a;
  a = 0;
  
  if(there_is_rotation)
  {
    r[0] = atof(argv[3]);
    r[1] = atof(argv[4]);
    r[2] = atof(argv[5]);
    a = atof(argv[6]) / 180.f * M_PI;
    
    there_is_rotation = (r[0] != 0 || r[1] != 0 || r[2] != 0) && (a != 0);
  }

  ObjectModel model;
  try
  {
    model.loadDirectory(model_dir, true);
  }catch(std::string ex)
  {
    ROS_ERROR("Error: %s", ex.c_str());
    return 3;
  }
  
  if(model.Faces.empty())
  {
    ROS_WARN("There are no faces in the given model");
    return 2;
  }
  
  cv::Mat im(480, 640, CV_8UC3);
  im = cvScalar(255,255,255);
 
  float f = model.Faces[0].A.at<float>(0,0);
  cv::Mat A = (cv::Mat_<float>(3, 3) << f, 0, 320, 0, f, 240, 0, 0, 1);
  cv::Mat cTo = DUtilsCV::Transformations::transl(0, 0, z);
  
  if(there_is_rotation)
  {
    cv::Mat axis(1, 3, CV_32F, r);
    cTo *= DUtilsCV::Transformations::rotvec(axis, a);
  }

  model.getVisualizationModel().draw(im, cTo, A);
  
  if(!output_img.empty()) cv::imwrite(output_img, im);
  
  DUtilsCV::GUI::showImage(im);

  return 0;

}



