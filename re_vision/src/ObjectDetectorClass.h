/** \file ObjectDetectorClass.h
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

#ifndef __OBJECT_DETECTOR_CLASS__
#define __OBJECT_DETECTOR_CLASS__

#include "ros/ros.h"
#include <string>
#include <vector>
#include "ObjectDetectorProvider.h"

class ObjectDetectorClass
{
public:

  ObjectDetectorClass();
  virtual ~ObjectDetectorClass();

  /**
   * Initiates the ObjectDetector node
   * @param argc
   * @param argv
   */
  void runObjectDetectorNode(int argc, char *argv[]);

  /**
   * Initiates the ObjectDetectorDummy node
   * @param argc
   * @param argv
   */
  void runObjectDetectorDummyNode(int argc, char *argv[],
    const std::vector<std::string> &local_paths);
    
protected:

  /** 
   * Parses the arguments given
   */
  void parseArgs(int argc, char *argv[]);

  /**
   * Checks the data and initiates the node
   */
  void initiateNode();
  
  /**
   * Runs ros::spin
   */
  void spinNode();

  /**
   * Creates a debug directory and returns its location
   * @param debug_dir location of created dir
   */
  void createDebugDir(std::string &debug_dir) const;

  /**
   * Loads a model of a poster whose location is fixed
   */
  void loadLocalModel(ObjectDetectorProvider &provider);

protected:

  struct tArguments
  {
    //std::string topic_name;
    //std::string color_model;
    std::string camera_model;
    bool debug_mode;
    bool visualization_mode;
  }; 
  
  tArguments m_arguments;
  std::string m_node_name;
  ros::NodeHandle *m_node_handle;  
  ObjectDetectorProvider *m_provider;

};

#endif


