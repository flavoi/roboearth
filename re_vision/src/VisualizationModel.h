/** \file VisualizationModel.h
 * \brief Visualization model of an object
 *
 * Class to store a general object visualization model
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


#ifndef __VISUALIZATION_MODEL__
#define __VISUALIZATION_MODEL__

#include <cv.h>

class VisualizationModel
{
public:

  VisualizationModel(){}
  virtual ~VisualizationModel(){}
  
  /**
   * Draws the model in the given image
   * @param img target image
   * @param cTo transformation from (c)amera to (o)bject
   * @param A intrinsic parameters of the camera that took image img
   * @param K distortion coefficient vector
   */
  virtual void draw(cv::Mat &img, const cv::Mat &cTo,
    const cv::Mat &A, const cv::Mat &K = cv::Mat()) const = 0;

};

#endif
