/** \file CameraBridgeFactory.cpp
 * \brief Factory of CameraBridge instances
 *
 * This class allows the creation of debugging CameraBridge instances
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
#include "CameraBridgeFactory.h"
#include <string>

// ----------------------------------------------------------------------------

const std::string CameraBridgeFactory::m_camera_names[NCameras] = 
{	
	"pike", "unibrain", "unibrain_monoslam", "sony_test", "gazebo_sim",
	"unibrain_rectified", "unibrain_bayer"
};

// ----------------------------------------------------------------------------

CameraBridge CameraBridgeFactory::Create(const std::string& model)
{

	if(model == ""){
		// default camera
		return CameraBridge();
		//return CameraBridge(CameraBridge::BW);
	
	}else if(model == m_camera_names[0]){ // pike
		// Pike camera
		return CameraBridge();
		//return CameraBridge(CameraBridge::BW);
		
	}else if(model == m_camera_names[1]){ // the unibrain wide angle lens camera
		// pixel units
		const float s = 2.f; // params are for 320x240, but images are at 640x480
		const float cx = 171.78f * s; //171.95f * s;
		const float cy = 127.41f  * s; //127.69f * s;
		const float f = 196.8214f * s; //195.7243f * s;
		const float k1 = -0.3421f;
		const float k2 = 0.1468f;
		const int w = 640;
		const int h = 480;
		
		//return CameraBridge(CameraBridge::BAYER_BG, w, h,
		//	cx, cy, f, f, k1, k2, 0.f, 0.f);
		return CameraBridge(w, h,
			cx, cy, f, f, k1, k2, 0.f, 0.f);
		
	}else if(model == m_camera_names[2]){ // unibrain (wide angle lens) but
		// with the monoslam configuration (320x240 with bayer corrected)
		
		const float s = 1.f; // params are for 320x240
		const float cx = 171.78f * s; //171.95f * s;
		const float cy = 127.41f  * s; //127.69f * s;
		const float f = 196.8214f * s; //195.7243f * s;
		const float k1 = -0.3421f;
		const float k2 = 0.1468f;
		const int w = 320;
		const int h = 240;
		
		//return CameraBridge(CameraBridge::BW, w, h,
		//	cx, cy, f, f, k1, k2, 0.f, 0.f);
		return CameraBridge(w, h,
			cx, cy, f, f, k1, k2, 0.f, 0.f);
 
  }else if(model == m_camera_names[3]) {
    // sony dsc-w30 photo camera, used to take the images for creating 3d models
    // it is used for testing purposes only
    
    // model images are undistorted
    const int w = 1632;
    const int h = 1224;
    const float cx = w/2.f;
    const float cy = h/2.f;
    const float f = 1790; // this depends on the image selected
    
    // the tester sends bw images
    //return CameraBridge(CameraBridge::BW, w, h, cx, cy, f, f);
    return CameraBridge(w, h, cx, cy, f, f);
  
  }else if(model == m_camera_names[4]) {
    // gazebo sim, custom parameters
    
    // model images are undistorted
    const int w = 640;
    const int h = 480;
    const float cx = 320.5f;
    const float cy = 240.5f;
    const float f = 320.f;
    //return CameraBridge(CameraBridge::BW, w, h, cx, cy, f, f);
    return CameraBridge(w, h, cx, cy, f, f);

  }else if(model == m_camera_names[5]) {
    // unibrain, but image is already rectified
    const float s = 2.f; // params are for 320x240, but images are at 640x480
		const float cx = 171.78f * s; //171.95f * s;
		const float cy = 127.41f  * s; //127.69f * s;
		const float f = 196.8214f * s; //195.7243f * s;
		const int w = 640;
		const int h = 480;
		
		return CameraBridge(w, h,
			cx, cy, f, f, 0.f, 0.f, 0.f, 0.f);
  
  }else if(model == m_camera_names[6]) {
    // unibrain, but the bayer patter and the distortion must 
    // be removed from the image
    // (this is a debugging camera)
    const float s = 2.f; // params are for 320x240, but images are at 640x480
		const float cx = 171.78f * s; //171.95f * s;
		const float cy = 127.41f  * s; //127.69f * s;
		const float f = 196.8214f * s; //195.7243f * s;
		const float k1 = -0.3421f;
		const float k2 = 0.1468f;
		const int w = 640;
		const int h = 480;
		
		return CameraBridge(CameraBridge::BAYER_BG, w, h, 
		  cx, cy, f, f, k1, k2, 0.f, 0.f);
  
	}else{
		throw std::string("Unknown camera");
	}
	
}

// ----------------------------------------------------------------------------

bool CameraBridgeFactory::IsValid(const std::string& model)
{
	if(model.empty()) return true;
	
	for(short i = 0; i < NCameras; ++i){
		if(model == CameraBridgeFactory::m_camera_names[i]){
			return true;
		}
	}
	return false;
}

