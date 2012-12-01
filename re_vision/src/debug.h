/** \file debug.h
 * \brief Debug functions
 *
 * Debug functions for ObjectDetection
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

#ifndef __D_DEBUG__
#define __D_DEBUG__

#include <opencv/cv.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include "geometry_msgs/Point.h"

using namespace std;

class debug
{
public:

  // Creates a VRML file with the axis of the given transformation
  static void saveTscene(const cv::Mat &rTo, 
    const std::vector<geometry_msgs::Point> &wP3d,
    const std::string &filename);

protected:

  // prints the VRML code for drawing a reference
  static void saveRef(fstream &f, const cv::Mat &wTr);
  
  // prints the VRML code for drawing a rectangle
  static void saveRectangle(fstream &f, const cv::Mat &wTr, float width, 
    float height);
  
  // prints the VRML code for drawing 3d points
  static void savePoints(fstream &f, 
    const std::vector<geometry_msgs::Point> &wP3d);

};



#endif
