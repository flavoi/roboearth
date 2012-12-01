/** \file VisualizationManager.h
 * \brief Allows to visualize detection images with ros
 * 
 * Class to send images by a ros topic to control visualization images
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

#ifndef __VISUALIZATION_MANAGER__
#define __VISUALIZATION_MANAGER__

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include "DUtils.h"

class VisualizationManager
{
public:
  /** 
   * Creates the visualization manager attached to the publisher it must
   * send the images through
   * @param pub publisher
   */
  VisualizationManager(ros::Publisher &pub);
  
  ~VisualizationManager();
  
  /** 
   * Sends the given image through the publishing topic
   * @param image
   * @param hold if true, image is frozen for a short time
   */
  void show(const cv::Mat &image, const std::string &text = "", 
    bool hold = false);

protected:
  /**
   * Puts text in the bottom part of the image
   * @param image
   * @param text
   */
  void putText(cv::Mat &image, const std::string &text,
    const cv::Scalar &color) const;

protected:
  ros::Publisher m_publisher;
  sensor_msgs::CvBridge m_bridge;
  
  bool m_holding; // true if current image is frozen
  DUtils::Timestamp m_unfreeze_time; // time until current image is frozen
};

#endif
