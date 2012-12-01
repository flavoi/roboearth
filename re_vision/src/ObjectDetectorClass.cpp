/** \file ObjectDetectorClass.cpp
 * \brief ObjectDetector functionality
 *
 * Code for ObjectDetector and ObjectDetectorDummy
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
#include <string>
#include <vector>

#include "ObjectDetectorClass.h"
#include "ObjectDetectorProvider.h"
#include "CameraBridge.h"
#include "CameraBridgeFactory.h"

#include "DUtils.h"

using namespace std;

// ---------------------------------------------------------------------------

ObjectDetectorClass::ObjectDetectorClass(): 
  m_node_handle(NULL), m_provider(NULL)
{
}

// ---------------------------------------------------------------------------

ObjectDetectorClass::~ObjectDetectorClass()
{ 
  delete m_provider;
  delete m_node_handle;
}

// ---------------------------------------------------------------------------

void ObjectDetectorClass::runObjectDetectorNode(int argc, char *argv[])
{
  m_node_name = "ObjectDetector";
  ros::init(argc, argv, m_node_name.c_str());

	parseArgs(argc, argv);
	initiateNode();
	
	spinNode();
}

// ---------------------------------------------------------------------------  

void ObjectDetectorClass::runObjectDetectorDummyNode(int argc, char *argv[],
  const std::vector<std::string> &local_paths)
{
  m_node_name = "ObjectDetectorDummy";
  ros::init(argc, argv, m_node_name.c_str());
  
  parseArgs(argc, argv);
	initiateNode();

	vector<string>::const_iterator sit;
	for(sit = local_paths.begin(); sit != local_paths.end(); ++sit)
	{
	  // load local model
	  std_msgs::StringPtr uri(new std_msgs::String);
    uri->data = *sit;
    m_provider->TopicNewModel(uri);
	}
	
	spinNode();
}

// ---------------------------------------------------------------------------  

void ObjectDetectorClass::parseArgs(int argc, char *argv[])
{
  m_arguments.camera_model = "";
	m_arguments.debug_mode = false;
	//m_arguments.topic_name = "/camera_info";
	//m_arguments.color_model = "bw";
	
	// parse arguments
	int c;
	while ((c = getopt (argc, argv, "vdc:")) != -1)
	{
		switch (c){
			case 'd': 
				m_arguments.debug_mode = true;
				break;
			
			case 'c':
				m_arguments.camera_model = optarg;
				break;
		  
		  case 'v':
		    m_arguments.visualization_mode = true;
		    break;
			
			// Not used
			//case 't':
			//  m_arguments.topic_name = optarg;
			//  break;
			
			// Not used
			//case 'm': 
			//  m_arguments.color_model = optarg;
			//  break;
		}
	}
	
	/*
	const string &s = m_arguments.color_model;
	if(s != "bw" && s != "rgb" && s != "bgr" && s != "bayer_bg" 
	  && s != "bayer_gb" && s != "bayer_rg" && s != "bayer_gr")
	{
	  ROS_WARN("Color model %s is not valid. Assuming %s", s.c_str(), "bayer_bg");
	  m_arguments.color_model = "bayer_bg";
	}
	*/
	
	if(!m_arguments.camera_model.empty())
  {
    if(!CameraBridgeFactory::IsValid(m_arguments.camera_model))
    {
      ROS_WARN("Camera %s is not known. Camera parameters will be retrieved "
        "from the camera info topic", m_arguments.camera_model.c_str() );
      m_arguments.camera_model = "";
    }
  }
	
}

// ---------------------------------------------------------------------------  

void ObjectDetectorClass::initiateNode()
{
	m_node_handle = new ros::NodeHandle;

	if(m_arguments.camera_model.empty())
	{
	  m_provider = new ObjectDetectorProvider(*m_node_handle);
	}
	else
	{	
    CameraBridge camera = CameraBridgeFactory::Create(m_arguments.camera_model);
    m_provider = new ObjectDetectorProvider(*m_node_handle, camera);
  }
  m_provider->init();

  std::string debug_dir = "";
  if(m_arguments.debug_mode) createDebugDir(debug_dir);

  m_provider->SetDebugMode(m_arguments.debug_mode, debug_dir);
  m_provider->SetVisualizationMode(m_arguments.visualization_mode);
  

	if(m_arguments.camera_model.empty())
		ROS_INFO("%s initiated, listening to topic for camera parameters", 
		  m_node_name.c_str());
	else
		ROS_INFO("%s initiated with predefined parameters of camera %s", 
		  m_node_name.c_str(), m_arguments.camera_model.c_str());

  if(m_arguments.debug_mode)
	  ROS_DEBUG("Debug mode is enabled under directory %s", debug_dir.c_str());
	else
    ROS_DEBUG("Debug mode is disabled");
}

// ---------------------------------------------------------------------------  

void ObjectDetectorClass::spinNode()
{
  ROS_INFO("%s is listening...", m_node_name.c_str());
  ros::spin();
}

// ---------------------------------------------------------------------------  

void ObjectDetectorClass::createDebugDir(std::string &debug_dir) const
{
  if(!DUtils::FileFunctions::DirExists("debug"))
    DUtils::FileFunctions::MkDir("debug");
  
  DUtils::Timestamp t(DUtils::Timestamp::CURRENT_TIME);
  debug_dir = "debug/debug_" + t.Format(true);
  
  DUtils::FileFunctions::MkDir(debug_dir.c_str());
}

// ---------------------------------------------------------------------------  


