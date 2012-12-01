/** \file generateMeta.cpp
 * \brief Generates a simple meta.xml file
 *
 * Standalone application for generating a simple meta.xml file for 3D models.
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
#include <sstream>
#include <vector>
#include <iomanip>

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

#include "../MetaFile.h"

using namespace std;

// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  if(argc < 5)
  {
    cout << "Usage: " << argv[0] << " <meta.xml file> <object name> "
      "<number of faces> <scale factor>" << endl;
    return 1;
  }
  
  string out_file = argv[1];
  string object_name = argv[2];
  int nfaces = atoi(argv[3]);
  float scale_factor = atof(argv[4]);

  MetaFile::MetaData data;
  
  data.Name = object_name;
  data.NFaces = nfaces;
  data.Type = "3D";
  data.Dimensions.Volume.Scale = scale_factor;
  
  MetaFile::saveFile(out_file, data);

  return 0;
}

// ---------------------------------------------------------------------------

