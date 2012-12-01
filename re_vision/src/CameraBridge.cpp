/** \file CameraBridge.cpp
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

#include "CameraBridge.h"
#include <opencv/cv.h>

#include "ros/ros.h"

// ---------------------------------------------------------------------------

CameraBridge::CameraBridge():
  m_image_type(CameraBridge::BW),
	m_there_is_calibration(false),
	m_there_is_distortion(false)
{
}

// ---------------------------------------------------------------------------

CameraBridge::CameraBridge(int w, int h,
	float cx, float cy, float fx, float fy):
	m_image_type(CameraBridge::BW),
	m_there_is_calibration(true),
	m_there_is_distortion(false),
	m_image_width(w),
	m_image_height(h)
{
	m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
		fx, 	0, 	cx,
		0, 	fy,	cy,
		0,	0,	1);
  
  m_distortion = cv::Mat::zeros(4,1, CV_32F);
}

// ---------------------------------------------------------------------------

CameraBridge::CameraBridge(int w, int h,
	float cx, float cy, float fx, float fy, 
	float k1, float k2, float p1, float p2):
	m_image_type(CameraBridge::BW),
	m_there_is_calibration(true),
	m_there_is_distortion(true),
	m_image_width(w),
	m_image_height(h)
{
	m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
		fx, 	0, 	cx,
		0, 	fy,	cy,
		0,	0,	1);
	
	m_distortion = (cv::Mat_<float>(4,1) << k1, k2, p1, p2);
	
	createMaps();
}

// ---------------------------------------------------------------------------

CameraBridge::CameraBridge(ImageType _imageType):
	m_image_type(_imageType), m_there_is_calibration(false),
	m_there_is_distortion(false)
{
}

// ---------------------------------------------------------------------------

CameraBridge::CameraBridge(ImageType _imageType, 
	int w, int h,
	float cx, float cy, float fx, float fy):
	m_image_type(_imageType),
	m_there_is_calibration(true),
	m_there_is_distortion(false),
	m_image_width(w),
	m_image_height(h)
{
	m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
		fx, 	0, 	cx,
		0, 	fy,	cy,
		0,	0,	1);
  
  m_distortion = cv::Mat::zeros(4,1, CV_32F);
}

// ---------------------------------------------------------------------------

CameraBridge::CameraBridge(ImageType _imageType, 
	int w, int h,
	float cx, float cy, float fx, float fy, 
	float k1, float k2, float p1, float p2):
	m_image_type(_imageType),
	m_there_is_calibration(true),
	m_there_is_distortion(true),
	m_image_width(w),
	m_image_height(h)
{
	m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
		fx, 	0, 	cx,
		0, 	fy,	cy,
		0,	0,	1);
	
	m_distortion = (cv::Mat_<float>(4,1) << k1, k2, p1, p2);
	
	createMaps();
}

// ---------------------------------------------------------------------------

void CameraBridge::SetParameters(int w, int h,
  float cx, float cy, float fx, float fy)
{
	m_image_width = w;
	m_image_height = h;
	
  m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
    fx, 	0, 	cx,
    0, 	fy,	cy,
    0,	0,	1);

  m_distortion = cv::Mat::zeros(4,1, CV_32F);
  
  m_there_is_calibration = true;
  m_there_is_distortion = false;
}

// ---------------------------------------------------------------------------

void CameraBridge::SetParameters(int w, int h,
  float cx, float cy, float fx, float fy,
  float k1, float k2, float p1, float p2)
{
	m_image_width = w;
	m_image_height = h;
	
  m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
    fx, 	0, 	cx,
    0, 	fy,	cy,
    0,	0,	1);

  m_distortion = (cv::Mat_<float>(4,1) << k1, k2, p1, p2);
	createMaps();
  
  m_there_is_calibration = true;
  m_there_is_distortion = true;
}

// ---------------------------------------------------------------------------

// DEPRECATED
void CameraBridge::SetParameters(ImageType _imageType, 
  int w, int h,
  float cx, float cy, float fx, float fy)
{
  m_image_type = _imageType;
	m_image_width = w;
	m_image_height = h;
	
  m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
    fx, 	0, 	cx,
    0, 	fy,	cy,
    0,	0,	1);

  m_distortion = cv::Mat::zeros(4,1, CV_32F);
  
  m_there_is_calibration = true;
  m_there_is_distortion = false;
}

// ---------------------------------------------------------------------------

// DEPRECATED
void CameraBridge::SetParameters(ImageType _imageType, 
  int w, int h,
  float cx, float cy, float fx, float fy,
  float k1, float k2, float p1, float p2)
{
  m_image_type = _imageType;
	m_image_width = w;
	m_image_height = h;
	
  m_intrinsic_parameters = (cv::Mat_<float>(3,3) <<
    fx, 	0, 	cx,
    0, 	fy,	cy,
    0,	0,	1);

  m_distortion = (cv::Mat_<float>(4,1) << k1, k2, p1, p2);
	createMaps();
  
  m_there_is_calibration = true;
  m_there_is_distortion = true;
}

// ---------------------------------------------------------------------------

void CameraBridge::createMaps()
{	
	cv::initUndistortRectifyMap(m_intrinsic_parameters, m_distortion,
		cv::Mat(), m_intrinsic_parameters, 
		cv::Size(m_image_width, m_image_height), CV_32F,
		m_remap_1, m_remap_2);
}

// ---------------------------------------------------------------------------
	
void CameraBridge::ConvertImage(cv::Mat &image) const
{
	cv::Mat aux, aux2;
	
	// fix colors (this is not probably used)
	switch(m_image_type){
		case BAYER_BG:
			cv::cvtColor(image, aux2, CV_BayerBG2RGB);
			break;
			
		case BAYER_GB:
			cv::cvtColor(image, aux2, CV_BayerGB2RGB);
			break;

		case BAYER_RG:
			cv::cvtColor(image, aux2, CV_BayerRG2RGB);
			break;

		case BAYER_GR:
			cv::cvtColor(image, aux2, CV_BayerGR2RGB);
			break;

		case RGB:
			cv::cvtColor(image, aux, CV_RGB2GRAY);
			break;
			
		case BGR:
			cv::cvtColor(image, aux, CV_BGR2GRAY);
			break;
		
		case BW:
			aux = image;
			break;
	}
	
	if(!aux2.empty()) 
	  cv::cvtColor(aux2, aux, CV_RGB2GRAY);
	
	// undistort image
	if(m_there_is_distortion){
	  if(aux.data == image.data) aux = aux.clone();
	  
	  //cv::undistort(aux, image, m_intrinsic_parameters, m_distortion);
		cv::remap( aux, image, m_remap_1, m_remap_2, cv::INTER_LINEAR, 
  	  cv::BORDER_REPLICATE );
	  
	}else
		image = aux;
	
	// smooth image
	cv::GaussianBlur(image, image, cv::Size(3,3), 0);
	
}

// ---------------------------------------------------------------------------

void CameraBridge::DistortPoint(int &x, int &y) const
{
	if(m_there_is_distortion && x >= 0 && x < m_image_width &&
	  y >= 0 && y < m_image_height)
	{
		int xx = x;
		int yy = y;
		x = (int)m_remap_1.at<float>(yy, xx);
		y = (int)m_remap_2.at<float>(yy, xx);
	}
}

// ---------------------------------------------------------------------------

void CameraBridge::DistortPoint(float x, float y, float& distx, float& disty) const
{
	if(m_there_is_distortion && x >= 0 && x < m_image_width &&
	  y >= 0 && y < m_image_height)
	{
		distx = m_remap_1.at<float>(y, x);
		disty = m_remap_2.at<float>(y, x);
	}else{
		distx = x;
		disty = y;
	}
}

// ---------------------------------------------------------------------------
