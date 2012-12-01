/** \file PlanarVisualizationModel.h
 * \brief Visualization model of a planar object
 *
 * Class to store a planar object visualization model
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

#ifndef __PLANAR_VISUALIZATION_MODEL__
#define __PLANAR_VISUALIZATION_MODEL__

#include "VisualizationModel.h"
#include "ObjectModel.h"
#include "MetaFile.h"
#include <cv.h>
#include <string>

class PlanarVisualizationModel: public VisualizationModel
{
public:
  
  /**
   * Creates the visualization model
   * @param faces faces present in the model. Only the image and the camera
   *   info is used
   * @param dim geometry and size information of the object and the faces
   * @note The reference system is assumed to be x pointing right, y pointing down
   *   and z going away the camera, with the origin in the center of the object
   */
  PlanarVisualizationModel(const std::vector<ObjectModel::Face> &faces, 
    const MetaFile::MetaData::tDimensions::tPlanar &dim);
    
  virtual ~PlanarVisualizationModel(){}
  
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

  /**
   * Draws only one face of the  model in the given image according to a 
   * homography
   * @param img target image
   * @param sHm homography to convert model points into img points
   * @param face_idx index of the face to draw
   */
  void draw(cv::Mat &img, const cv::Mat &sHm, unsigned int face_idx) const;

protected:

  /// Images for visualization (colour and bw)
  std::vector<cv::Mat> m_face_images, m_face_images_bw;
  
  /// Inverse matrix of the intrinsic parameters of the cameras that took
  /// each image (double format)
  std::vector<cv::Mat> m_inv_A;
  
  /// Transformations from origin of object to faces, dimensions of the object
  /// and dimensions of the faces
  MetaFile::MetaData::tDimensions::tPlanar m_dim;
  
};

#endif
