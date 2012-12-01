/** \file debug.cpp
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

#include <opencv/cv.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include "debug.h"

#include "geometry_msgs/Point.h"

// -------------------------------------------------------------------------

void debug::saveTscene(const cv::Mat &rTo, 
  const std::vector<geometry_msgs::Point> &points3d,
  const std::string &filename)
{
  fstream f(filename.c_str(), ios::out);

	f << "#VRML V2.0 utf8" << endl << endl;

	// axes and viewpoint
	f << "Viewpoint { position 0 0 -3 " << endl
		<< "orientation 1 0 0 -3.1415926535897931 # towards 0 0 0  tilt -180 degrees" << endl
		<< "fieldOfView 0.78539816339744828 }" << endl << endl;
  
  cv::Mat wTr = cv::Mat::eye(4,4,CV_64F);
	saveRef(f, wTr);
	saveRef(f, wTr * rTo);
	saveRectangle(f, wTr * rTo, 1.0, 1.82);
	savePoints(f, points3d);

	f.close();
}

// -------------------------------------------------------------------------

void debug::savePoints(fstream &f, 
  const std::vector<geometry_msgs::Point> &wP3d)
{
  for(unsigned int i = 0; i < wP3d.size(); ++i)
  {
    double x = wP3d[i].x;
    double y = wP3d[i].y;
    double z = wP3d[i].z;
    
    f << "Transform {" << endl
      << "translation " << x << " " << y << " " << z << endl
	    << "  children[" << endl
		  << "    Shape{" << endl
		  << "      geometry Sphere { radius 0.01 }" << endl
		  << "    }" << endl
		  << "  ]" << endl
		  << "}" << endl << endl;
  }
}
    
// -------------------------------------------------------------------------

void debug::saveRectangle(fstream &f, const cv::Mat &wTr, float width, 
  float height)
{
  const float w2 = width / 2.f;
  const float h2 = height / 2.f;
  
  cv::Mat rP = (cv::Mat_<double>(4, 4) << 
    -w2,  w2, w2, -w2,
    -h2, -h2, h2,  h2,
      0,   0,  0,   0,
      1,   1,  1,   1);

  cv::Mat wP = wTr * rP; 
  
  double x[4], y[4], z[4];
  stringstream s[4];
  
  for(int i = 0; i < 4; ++i)
  {
    x[i] = wP.at<double>(0,i) / wP.at<double>(3,i);
    y[i] = wP.at<double>(1,i) / wP.at<double>(3,i);
    z[i] = wP.at<double>(2,i) / wP.at<double>(3,i);
    
    s[i] << x[i] << " " << y[i] << " " << z[i];
  }
  
  // lines
  f << "Shape {" << endl
		<< "geometry IndexedLineSet {" << endl
		<< "color Color {" << endl
        <<            "color [" << endl
                       << "1.0 1.0 1.0," << endl
                       << "1.0 1.0 1.0," << endl
                       << "1.0 1.0 1.0," << endl
                       << "1.0 1.0 1.0," << endl
                    << "]" << endl
                << "}" << endl
                << "coord Coordinate {" << endl
                    << "point [" << endl
                      << s[0].str() << "," << endl
                      << s[1].str() << "," << endl
                      << s[2].str() << "," << endl
                      << s[3].str() << "," << endl
                    << "]" << endl
                << "}" << endl
                << "coordIndex [" << endl
                    << "0, 1, 2, 3, 0" << endl
                << "]" << endl
            << "}" << endl
        << "}" << endl << endl;
  
  /* // plane
  // front
  f << "Shape {" << endl << "geometry IndexedFaceSet {"
  << endl << "coord Coordinate { point [" << endl;
  for(short i = 0; i < 4; i++)
  {
    double x = wP.at<double>(0,i) / wP.at<double>(3,i);
    double y = wP.at<double>(1,i) / wP.at<double>(3,i);
    double z = wP.at<double>(2,i) / wP.at<double>(3,i);
    
    f << x << " " << y << " " << z << ",";
  }
  f << " ] }";
  f << "coordIndex [0,  1, 2, 3, 0] } }" << endl;

  // and back
  f << "Shape {" << endl << "geometry IndexedFaceSet {"
    << endl << "coord Coordinate { point [" << endl;
  for(short i = 0; i < 4; i++)
  {
    double x = wP.at<double>(0,i) / wP.at<double>(3,i);
    double y = wP.at<double>(1,i) / wP.at<double>(3,i);
    double z = wP.at<double>(2,i) / wP.at<double>(3,i);
    
    f << x << " " << y << " " << z << ",";
  }
  f << " ] }";
  f << "coordIndex [3, 2, 1, 0, 3] } }" << endl;
  */
  
}

// -------------------------------------------------------------------------

void debug::saveRef(fstream &f, const cv::Mat &wTr)
{
  const float L = 0.5; // axis length;

  cv::Mat rP = (cv::Mat_<double>(4, 4) << 
    0, L, 0, 0,
    0, 0, L, 0,
    0, 0, 0, L,
    1, 1, 1, 1);

  cv::Mat wP = wTr * rP;
  
  double ox = wP.at<double>(0,0) / wP.at<double>(3,0);
  double oy = wP.at<double>(1,0) / wP.at<double>(3,0);
  double oz = wP.at<double>(2,0) / wP.at<double>(3,0);
  
  double rx = wP.at<double>(0,1) / wP.at<double>(3,1);
  double ry = wP.at<double>(1,1) / wP.at<double>(3,1);
  double rz = wP.at<double>(2,1) / wP.at<double>(3,1);
  
  double gx = wP.at<double>(0,2) / wP.at<double>(3,2);
  double gy = wP.at<double>(1,2) / wP.at<double>(3,2);
  double gz = wP.at<double>(2,2) / wP.at<double>(3,2);
  
  double bx = wP.at<double>(0,3) / wP.at<double>(3,3);
  double by = wP.at<double>(1,3) / wP.at<double>(3,3);
  double bz = wP.at<double>(2,3) / wP.at<double>(3,3);

  stringstream os, rs, gs, bs;
  os << ox << " " << oy << " " << oz;
  rs << rx << " " << ry << " " << rz;
  gs << gx << " " << gy << " " << gz;
  bs << bx << " " << by << " " << bz;
  
  f << "Shape {" << endl
		<< "geometry IndexedLineSet {" << endl
		<< "color Color {" << endl
        <<            "color [" << endl
                       << "1.0 0.0 0.0," << endl
                        << "1.0 0.0 0.0," << endl
                        << "0.0 1.0 0.0," << endl
                        << "0.0 1.0 0.0," << endl
                        << "0.0 0.0 1.0," << endl
                        << "0.0 0.0 1.0," << endl
                    << "]" << endl
                << "}" << endl
                << "coord Coordinate {" << endl
                    << "point [" << endl
                      << os.str() << "," << endl
                      << rs.str() << "," << endl
                      << os.str() << "," << endl
                      << gs.str() << "," << endl
                      << os.str() << "," << endl
                      << bs.str() << "," << endl
                    << "]" << endl
                << "}" << endl
                << "coordIndex [" << endl
                    << "0, 1, -1," << endl
                    << "2, 3, -1," << endl
                    << "4, 5, -1," << endl
                << "]" << endl
            << "}" << endl
        << "}" << endl << endl;
}


// -------------------------------------------------------------------------

