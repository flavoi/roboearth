/** \file Mask.cpp
 * \brief Class for managing image masks
 *
 * Class for creating and operating with image masks.
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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Mask.h"

// ---------------------------------------------------------------------------

Mask::Mask(const char *filename)
{
  m_original_mask = cv::imread(filename, 0);
  m_current_mask = m_original_mask.clone();
}

// ---------------------------------------------------------------------------
  
Mask::Mask(const cv::Mat &mask)
{
  m_original_mask = mask.clone();
  m_current_mask = m_original_mask.clone();
}

// ---------------------------------------------------------------------------
  
Mask::~Mask()
{
}

// ---------------------------------------------------------------------------

void Mask::shrink(int margin)
{
  if(margin > 0)
  {
    cv::Mat erosion = cv::Mat::ones(margin*2+1, margin*2+1, CV_8U);
    cv::Mat aux;
    cv::erode(m_current_mask, aux, erosion);
    m_current_mask = aux;
  }
}

// ---------------------------------------------------------------------------
  
void Mask::reset()
{
  m_current_mask = m_original_mask.clone();
}

// ---------------------------------------------------------------------------

void Mask::maskKeyPoints(std::vector<cv::KeyPoint> &keys)
{
  int deleted = 0;
  const int N = keys.size();
  cv::KeyPoint aux;
  
  int i = 0;
  while(i < N - deleted)
  {
    if(!m_current_mask.at<unsigned char>(keys[i].pt.y, keys[i].pt.x))
    {
      // move to the last position
      aux = keys[ N - 1 - deleted ];
      keys[ N - 1 - deleted ] = keys[i];
      keys[i] = aux;
      ++deleted;
    }
    else
    {
      ++i;
    }
  }

  if(deleted) keys.resize(N - deleted);
}

// ---------------------------------------------------------------------------

void Mask::getPointsOutsideMask(const std::vector<cv::KeyPoint> &keys, 
    std::vector<unsigned int> &i_out) const
{
  i_out.clear();
  
  for(unsigned int i = 0; i < keys.size(); ++i)
  {
    if(!test(keys[i].pt.x, keys[i].pt.y)) i_out.push_back(i);
  }
  
}

// ---------------------------------------------------------------------------


