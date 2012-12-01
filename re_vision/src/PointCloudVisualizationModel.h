/** \file PointCloudVisualizationModel.h
 * \brief Visualization model of a 3d object composed of 3d points
 *
 * Class to store a 3d object visualization model composed of 3d points
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

#ifndef __POINT_CLOUD_VISUALIZATION_MODEL__
#define __POINT_CLOUD_VISUALIZATION_MODEL__

#include "VisualizationModel.h"
#include <cv.h>
#include <string>
#include <vector>

#include "DVision.h"

class PointCloudVisualizationModel: public VisualizationModel
{
public:
  
  /**
   * Creates the visualization model
   * @param filename ply filename
   */
  PointCloudVisualizationModel(const std::string &filename);
    
  virtual ~PointCloudVisualizationModel(){}
  
  /**
   * Draws the model in the given image
   * @param img target image
   * @param cTo transformation from (c)amera to (o)bject
   * @param A intrinsic parameters of the camera that took image img
   * @param K distortion coefficient vector
   * @see VisualizationModel::draw
   */
  void draw(cv::Mat &img, const cv::Mat &cTo, const cv::Mat &A,
    const cv::Mat &K = cv::Mat()) const;

protected:

  /**
   * Creates m_oP from m_plypoints
   */
  void convertPLY2Mat();
  
  /**
   * Calculates the dimensions of the object according to m_oP, and stores
   * the result in m_dim#
   */
  void calculateDimensions();

protected:

  /// Point cloud
  std::vector<DVision::PMVS::PLYFile::PLYPoint> m_plypoints;
  
  /// Point cloud in homogeneous coordinates:
  /// 4xN double matrix in the object reference
  cv::Mat m_oP;
  
  /// Dimensions
  float m_dimx, m_dimy, m_dimz;
  
};

#endif
