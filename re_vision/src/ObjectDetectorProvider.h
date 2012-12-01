/** \file ObjectDetectorProvider.h
 * \brief Provides the service and topics supported by ObjectDetector
 *
 * Class to provide the SearchFor (object) service and to listen to the
 * new_model topic
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

#ifndef __OBJECT_DETECTOR_PROVIDER__
#define __OBJECT_DETECTOR_PROVIDER__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include "re_vision/SearchFor.h"
#include "re_msgs/DetectedObject.h"
#include "CameraBridge.h"
#include <vector>
#include <string>

#include "ObjectModel.h"
#include "ObjectDetectorMethod.h"
#include "VisualizationManager.h"

#include <opencv/cv.h>


using namespace std;

class ObjectDetectorProvider
{
public:

  /** 
   * Creates the provider with the name of the topic to listen to to obtain
   * the camera parameters
   * @param node_handle
   */
  ObjectDetectorProvider(const ros::NodeHandle& node_handle);
  
  /**
   * Creates the provider with a predefined camera
   * @param node_handle
   * @param camera predefined camera parameters
   */
  ObjectDetectorProvider(const ros::NodeHandle& node_handle,
    const CameraBridge &camera);

	virtual ~ObjectDetectorProvider();

	void init();

  /**
   * Callback function of the SearchFor service
   */
	bool ServiceSearchFor(re_vision::SearchFor::Request  &req,
		re_vision::SearchFor::Response &res );
	
	/** 
	 * Callback function of the new_model topic
	 */	
  void TopicNewModel(const std_msgs::String::ConstPtr& msg);
	  
  /**
   * Callback function of the camera_info topic
   */
  void TopicCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& caminfo);
	  
  /** 
   * Sets the debug mode
   * @param onoff debug mode iif true
   * @param dir if given, directory to store debug files (must exist)
   */
  void SetDebugMode(bool onoff, const std::string &dir = "");
  
  /**
   * Sets the visualization mode
   * @param onoff
   */
  void SetVisualizationMode(bool onoff);
  
protected:
	// ROS-related functions
	
	cv::Mat getImage(const re_vision::SearchFor::Request &req);

	void showRequestInformation(const re_vision::SearchFor::Request &req,
		const cv::Mat &image) const;
		
	//void showImage(const cv::Mat &image) const;
	
	void processRequest(const re_vision::SearchFor::Request  &req,
		const cv::Mat &image, re_vision::SearchFor::Response &res);

	void emptyResponse(const re_vision::SearchFor::Request &req,
	  re_vision::SearchFor::Response &res) const;

protected:
	// Detector-related functions
 	
 	/**
 	 * Instantiates the object detection algorithms
 	 */
	void createDetectionAlgorithms();
	
	/**
	 * Adds the model located in the given path to the list of models
	 * @param path
	 */
	void learnNewModel(const std::string &path);
	
	/**
	 * Removes the given object from the list of recognizable objects
	 * @param name
	 */
	void removeModel(const std::string &name);

  /**
   * Distorts the 2d points detected in the rectified image to match the
   * location of the points in the original input image. 
   * This also removes those points whose distorted (x,y) end up outside
   * the image
   * @param detections detections
   * @param W
   * @param H: width and height of the input image
   */
	void rectifyDetections(std::vector<re_msgs::DetectedObject> &detections,
	  int W, int H, int max_points_per_object) const;
	  
	//void removeUnknownObjects(const std::vector<std::string> &objects,
	//	std::vector<std::string> &filtered) const;

	//virtual void initLearning() = 0;
	//virtual void finishLearning() = 0;
	
	void detectObjects(const std::vector<std::string> &objects,
			     const cv::Mat &image, int max_points_per_object,
			     std::vector<re_msgs::DetectedObject*> &ret);

  /**
   * Removes from detection some points if there are more than max_points
   * @param detection
   * @param max_points
   */
  void removeSomePoints(re_msgs::DetectedObject& detection,
    int max_points) const;

	inline bool debugMode() const { return m_debug; }
	
	// Returns a reference to the camera
	inline const CameraBridge& getCamera() const 
	{
		return m_camera;
	}

	void getValidObjects(const std::vector<std::string> &objects,
		std::vector<std::string>& valid_objects,
		std::vector<re_msgs::DetectedObject> &detections,
		std::vector<re_msgs::DetectedObject *> &pointers) const;

	/*
protected:
	

	// This function object is executed by another thread.
	// It must set m_ready = true when finished
	struct asyncLearner 
	{
		asyncLearner(ObjectDetectorProvider &parent): 
			m_parent(parent){}
				
		void operator()();
		
		private:
			ObjectDetectorProvider& m_parent;
	};
	

	void asyncLearnModels();
	*/
  
protected:
	ros::NodeHandle m_node_handle;
	ros::ServiceServer m_service;
	ros::Subscriber m_new_model_sub;
	ros::Subscriber *m_camera_info_sub;
	ros::Publisher m_visualization_pub;
	sensor_msgs::CvBridge m_bridge;
	

  //std::string m_camera_info_topic;
	CameraBridge m_camera;
	bool m_camera_info_got;
	
	//bool m_ready;
	bool m_debug;
	
	// visualization flag
	VisualizationManager *m_visualizer;
	bool m_visualization;

	//std::vector<ParsedObjectInformation> m_object_list;
	
	// Map object_name -> object_model
	typedef std::map<std::string, ObjectModel*> tModelMap;
	tModelMap m_models;
	
	// Map object_type -> object_detection_algorithm
	typedef std::map<std::string, ObjectDetectorMethod*> tAlgorithmMap;
	tAlgorithmMap m_algorithms;
  
};

#endif

