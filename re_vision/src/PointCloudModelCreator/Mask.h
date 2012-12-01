/** \file Mask.h
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

#ifndef __D_MASK__
#define __D_MASK__

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

class Mask
{
public:

  /**
   * Creates the mask by loading a mask file
   * @param filename filename of mask file
   */
  Mask(const char *filename);
  
  /**
   * Creates the mask by loading a mask image
   * @param mask
   */
  Mask(const cv::Mat &mask);
  
  virtual ~Mask();
  
  /**
   * Makes the current mask smaller by deleting those pixels whose distance to 
   * the edge is smaller than margin
   * @param margin
   */
  void shrink(int margin); 
  
  /**
   * Says whether there is some mask
   * @return true iff there is no mask
   */
  inline bool empty() const;
  
  /**
   * Resets the mask to its original state
   */
  void reset();
  
  /**
   * Removes from keys those keypoints which are not inside the mask
   * @param keys
   * @note this function may change the order of the keypoints
   */
  void maskKeyPoints(std::vector<cv::KeyPoint> &keys);
  
  /**
   * Returns the indices of the given keys that are not inside the mask
   * @param keys
   * @param i_out indices of keys not inside the mask
   */
  void getPointsOutsideMask(const std::vector<cv::KeyPoint> &keys, 
    std::vector<unsigned int> &i_out) const;

  /**
   * Returns true if the pixel given is inside the mask,
   * false otherwise, or if the pixel is not in the image
   */
  inline bool test(int x, int y) const
  { 
    return x >= 0 && x < m_current_mask.cols &&
      y >= 0 && y < m_current_mask.rows &&
      m_current_mask.at<unsigned char>(y, x) != 0;
  }

protected:
  
  /// Original mask
  cv::Mat m_original_mask;
  
  /// Working mask
  cv::Mat m_current_mask;

};

// ---------------------------------------------------------------------------

inline bool Mask::empty() const
{
  return this->m_original_mask.empty();
}

// ---------------------------------------------------------------------------

#endif
