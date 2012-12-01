/** \file CameraBridgeFactory.h
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

#ifndef __CAMERA_BRIDGE_FACTORY__
#define __CAMERA_BRIDGE_FACTORY__

#include "CameraBridge.h"
#include <string>

class CameraBridgeFactory
{
	public:
		// Returns the camera parameters of a predefined camera.
		// If no model is given, a bw rectified camera is assumed
		static CameraBridge Create(const std::string& model = "");
	
		// Returns if the given camera is valid (or empty)
		static bool IsValid(const std::string& model);
	
	protected:
		// Supported camera names
		static const int NCameras = 7;
		static const std::string m_camera_names[NCameras];
};

#endif

