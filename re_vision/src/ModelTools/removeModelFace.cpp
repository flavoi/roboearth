/** \file removeModelFace.cpp
 * \brief Removes some face from a local object model
 *
 * removeModelFace allows to delete a face of an object model. This is useful
 * when the model created by Bundle+PMVS contains some erroneous face
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

// Usage: removeModelFace <model_dir> <face_idx>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../ObjectModel.h"
#include "../MetaFile.h"

#include "ros/ros.h"
#include "DUtils.h"
#include "DUtilsCV.h"

#include <stdio.h>

using namespace std;

// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "removeModelFace");
  
  if(argc < 2)
  {
    cout << "Usage: " << argv[0] << " <model dir> <face idx>" << endl;
    return 0;
  }

  string model_dir = argv[1];
  int face_idx = atoi(argv[2]);
  
  // check model
  if(!ObjectModel::checkDirectory(model_dir))
  {
    ROS_ERROR("Directory %s does not seem to contain a valid object model",
      model_dir.c_str());
    return 1;
  }
  
  // check index
  ObjectModel model(model_dir, false);
  
  if(model.Faces.empty())
  {
    ROS_WARN("Object %s does not contain any face", model.Name.c_str());
    return 0;
  }
  
  if(face_idx < 0 || face_idx >= (int)model.Faces.size())
  {
    ROS_WARN("Face index out of bounds. Object %s has %d faces (0 - %d)",
      model.Name.c_str(), model.Faces.size(), model.Faces.size() - 1);
    return 2;
  }
  
  vector<string> extensions;
  extensions.push_back(".key.gz");
  extensions.push_back(".ply");
  extensions.push_back(".png");
  extensions.push_back(".txt");
  
  for(int idx = face_idx; idx < (int)model.Faces.size()-1; ++idx)
  {
    // mv face_%03d .key.gz / .ply / .png / .txt one index backwards
    stringstream source, target;
    source << model_dir << "/face_" << setw(3) << setfill('0') << idx+1;
    target << model_dir << "/face_" << setw(3) << setfill('0') << idx;
    
    for(unsigned int j = 0; j < extensions.size(); ++j)
    {
      rename((source.str() + extensions[j]).c_str(), 
        (target.str() + extensions[j]).c_str() );
    }
  }
  
  // delete face_%03d with index |model.Faces|-1
  stringstream source;
  source << model_dir << "/face_" << setw(3) << setfill('0') 
    << model.Faces.size() - 1;

  for(unsigned int j = 0; j < extensions.size(); ++j)
  {
    remove((source.str() + extensions[j]).c_str());
  }
  
  // update meta
  ROS_INFO("Updating meta data...");
  
  MetaFile::MetaData data;
  string meta_file = model_dir + "/meta.xml";
  MetaFile::readFile(meta_file, data);
  
  data.NFaces--;
  if(data.Type == "planar")
  {
    data.Dimensions.Planar.Faces.erase(
      data.Dimensions.Planar.Faces.begin() + face_idx);
  }
  
  MetaFile::saveFile(meta_file, data);

  ROS_INFO("Done. Face %d of object %s has been deleted", face_idx, 
    model.Name.c_str());
}

// ---------------------------------------------------------------------------


