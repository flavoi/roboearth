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
 * Copyright (C) 2011 by <a href="mailto:riazuelo@unizar.es">Luis Riazuelo Latas</a>, University of Zaragoza
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
 * \author Luis Riazuelo Latas
 * \version 1.2
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

#include "ros/ros.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_pointcloud_model");
  ros::NodeHandle n;

  int retVal;
  std::string system_command;
  std::string command_create_model;


  if(argc < 6)
  {
    ROS_WARN("Usage: createPlanarModel <image dir> <model dir> "
      "<name model> <output model dir> <surf threshold>");
    return 1;
  }
  
  std::string ImageDir = argv[1];
  std::string ModelDir = argv[2];
  std::string Name = argv[3];
  std::string ModelOutDir = argv[4];
  std::string SurfThreshold = argv[5];

  std::string path_re_vision;

  FILE * find_file = popen("rospack find re_vision", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;

  path_re_vision = command_find;

  command_create_model = path_re_vision + "/bin"; 

  ROS_INFO("Ready to create pointcloud model.");

  system_command ="cd " + command_create_model + "; ./createModel.sh " + ImageDir + " " + ModelDir + " " + Name + " " + ModelOutDir + " " + SurfThreshold;

  retVal = system(system_command.c_str());

  return retVal;
}

