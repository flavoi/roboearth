/** \file VisualizationManager.cpp
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

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include "VisualizationManager.h"

#include "DUtils.h"

// --------------------------------------------------------------------------

VisualizationManager::VisualizationManager(ros::Publisher &pub):
  m_publisher(pub)
{
}

// --------------------------------------------------------------------------

VisualizationManager::~VisualizationManager()
{
}

// --------------------------------------------------------------------------

void VisualizationManager::show(const cv::Mat &image, 
  const std::string &text, bool hold)
{
  const float HOLDING_TIME = 2.f; // seconds
  const cv::Scalar HOLDING_COLOR(0, 255, 0);
  const cv::Scalar NORMAL_COLOR(255, 255, 255);

  // check if we can update the current image
  bool update_image = false;
  
  DUtils::Timestamp current_time(DUtils::Timestamp::CURRENT_TIME);
  
  if(hold)
  {
    // the given image has a higher priority
    m_holding = true;
    m_unfreeze_time = current_time + HOLDING_TIME;
    update_image = true;
  }
  else if(m_holding)
  {
    // check current time
    if(current_time >= m_unfreeze_time)
    {
      m_holding = false;
      update_image = true;
    }
  }
  else
    update_image = true;


  if(update_image)
  {
    cv::Mat img;
    
    if(image.channels() == 3)
      img = image.clone();
    else
      cvtColor(image, img, CV_GRAY2BGR);
    
    if(!text.empty()) putText(img, text, 
      (hold ? HOLDING_COLOR : NORMAL_COLOR));
    
    IplImage iplimg(img);
    
	  try{
		  sensor_msgs::Image::Ptr ros_img_ptr = 
		    m_bridge.cvToImgMsg(&iplimg, "passthrough");
		  sensor_msgs::Image msg = sensor_msgs::Image(*ros_img_ptr);
      
      m_publisher.publish(msg);
      
	  }catch (sensor_msgs::CvBridgeException error){
		  ROS_WARN("Error sending image for visualization");
	  }
	}

}

// --------------------------------------------------------------------------

void VisualizationManager::putText(cv::Mat &image, const std::string &text,
  const cv::Scalar &color) const
{ 
  const int face = cv::FONT_HERSHEY_COMPLEX;
  const double scale = 2.f;
  const int thickness = 2;
  const int linetype = 8;
  const bool bottomleftorigin = false;
  cv::Point origin;
  int baseline = 0;
  
  cv::Size size = cv::getTextSize(text, face, scale, thickness, &baseline);
  
  origin.y = image.rows - size.height;
  origin.x = (image.cols - size.width)/2;
  
  cv::putText(image, text, origin, face, scale, color, thickness, linetype,
    bottomleftorigin);
}

// --------------------------------------------------------------------------

