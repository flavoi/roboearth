/** \file Surf3DDetector.h
 * \brief Detection algorithm for 3D objects
 *
 * ObjectDetectorMethod for 3D textured objects using SURF
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


#ifndef __SURF_3D_DETECTOR__
#define __SURF_3D_DETECTOR__

#include "ObjectModel.h"
#include "ObjectDetectorMethod.h"
#include "re_msgs/DetectedObject.h"

#include "DVision.h"
#include <vector>

typedef DVision::SurfSet SurfSet;

// Set to 0 not to compile the debugging functions
#define VOLUME_DEBUG_ENABLED 1

class Surf3DDetector: public ObjectDetectorMethod
{
public:

#if VOLUME_DEBUG_ENABLED
  Surf3DDetector(): m_detect_counter(0){}
#else
  Surf3DDetector(){}
#endif

  virtual ~Surf3DDetector(){}

  /**
   * Tries to detect in the given data the given model. Results are stored
   * in detection
   * @param data
   * @param model model to search for
   * @param detection results
   * @see ObjectDetectorMethod::detect
   */
  void detect(ObjectDetectorMethod::DetectionData &data, 
    ObjectModel &model,
    re_msgs::DetectedObject &detection);

protected:

  /** 
   * Given the correspondences between the scene and the model, tries
   * to detect the object with pnp
   * @param scene_surfset surfs from the original image
   * @param model model checked
   * @param face face of the model checked
   * @param model_indices indices of model keypoints matched
   * @param scene_indices indices of scene keypoints matched
   * @param distances distances between the matched keypoint descriptors
   * @param data common detection data
   * @param maxReprojectionError max reprojection error to accept a reprojected
   *   point as an inlier
   * @param doReChecking if true, the algorithm is run again with the inliers
   *   got in the first iteration and with a smaller maxReprojectionError
   * @param detection place to put the results in
   * @return true iif the model can be detected
   */
  bool detectWithPnP(const SurfSet &scene_surfset,
    ObjectModel &model,
    ObjectModel::Face &face, const std::vector<int> &model_indices,
    const std::vector<int> &scene_indices, const std::vector<float> &distances,
    const ObjectDetectorMethod::DetectionData &data,
    const double maxReprojectionError, bool doReChecking,
    re_msgs::DetectedObject &detection) ;

protected:

  #if VOLUME_DEBUG_ENABLED
    int m_detect_counter; /// times ::detect has been called
    std::string m_debug_prefix; /// prefix for debug files
  #endif

};

#endif
