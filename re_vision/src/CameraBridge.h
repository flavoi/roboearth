/** \file CameraBridge.h
 * \brief Camera-dependent functions to rectify images
 *
 * A CameraBridge puts together the intrinsic and distortion parameters used
 * by a specific camera, to transform images into undistorted gray images.
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

#ifndef __CAMERA_BRIDGE__
#define __CAMERA_BRIDGE__

#include <cv.h>

class CameraBridge
{
	public:
		enum ImageType
		{
			BW,  // used for those cases where the conversion is done by ros
			RGB, // DEPRECATED
			BGR, // DEPRECATED
			// used when we have to remove the bayer pattern
			BAYER_BG, 
			BAYER_GB,
			BAYER_RG,
			BAYER_GR
		};
	
public:

  CameraBridge();
  
  CameraBridge(int w, int h,
		float cx, float cy, float fx, float fy);

  CameraBridge(int w, int h,
		float cx, float cy, float fx, float fy,
		float k1, float k2, float p1, float p2);

  void SetParameters(int w, int h,
		float cx, float cy, float fx, float fy);

  void SetParameters(int w, int h,
		float cx, float cy, float fx, float fy,
		float k1, float k2, float p1, float p2);

	CameraBridge(ImageType _imageType);

	CameraBridge(ImageType _imageType, 
		int w, int h,
		float cx, float cy, float fx, float fy);

	CameraBridge(ImageType _imageType, 
		int w, int h,
		float cx, float cy, float fx, float fy,
		float k1, float k2, float p1, float p2);

  // Changes the camera parameters
  void SetParameters(ImageType _imageType, 
		int w, int h,
		float cx, float cy, float fx, float fy);

  void SetParameters(ImageType _imageType, 
		int w, int h,
		float cx, float cy, float fx, float fy,
		float k1, float k2, float p1, float p2);
	
	// Converts a raw image into a rectified image
	void ConvertImage(cv::Mat &image) const;

	// Applies distortion to an undistorted point
	void DistortPoint(int &x, int &y) const;

	// The same as above
	void DistortPoint(float x, float y, float& distx, float& disty) const;

	// Returns whether there is calibration
	inline bool ThereIsCalibration() const 
	{
		return m_there_is_calibration;
	}
	
	// Returns whether there is distortion
	inline bool ThereIsDistortion() const
	{
	  return m_there_is_distortion;
	}

	// Returns a read-only version of the intrinsic parameters
	inline const cv::Mat& GetIntrinsicParameters() const
	{
		return m_intrinsic_parameters;
	}
	
	// Returns a read-only version of the distortion parameters
	inline const cv::Mat& GetDistortionParameters() const
	{
		return m_distortion;
	}
	
	// Returns trye if the camera uses a bayer pattern
	inline bool usesBayer() const
	{
	  return
	    m_image_type == BAYER_BG ||
			m_image_type == BAYER_GB ||
			m_image_type == BAYER_RG ||
			m_image_type == BAYER_GR;
	}

protected:
	
	// Creates un/distortion maps
	void createMaps();
			
protected:

	ImageType m_image_type; // DEPRECATED. It is ignored
	bool m_there_is_calibration;
	cv::Mat m_intrinsic_parameters; // 3x3 matrix
	cv::Mat m_distortion; // 4x1 vector

	// maps to un/distort points
	bool m_there_is_distortion;
	cv::Mat m_remap_1, m_remap_2;

	int m_image_width;
	int m_image_height;

};

#endif
