/** \file loadModelIntoVSLam.cpp
 * \brief Sends a model to the VSlam system
 *
 * Sends a model to the VSlam system, so that the model can be uploaded to
 * the RoboEarth database
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

/**
 * Usage: loadModelIntoVSlam <model dir>
 *
 * model dir: directory where the model is stored
 *
 */
 
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <stdlib.h>

#include "../ObjectModel.h"

using namespace std;

// ---------------------------------------------------------------------------

static const std::string topic_name = "re_vslam/load_model";

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "loadModelIntoVSlam");

  if(argc < 2)
  {
    ROS_WARN("Usage: %s <model dir>", argv[0]);
    return 1;
  }
  
  string model_dir = argv[1];
  
  if(model_dir.empty())
  {
    ROS_ERROR("No model directory given");
    return 2;
  }
  
  char *c_abspath = realpath(model_dir.c_str(),  NULL);
  if(!c_abspath)
  {
    ROS_ERROR("Could not get the absolute path of \"%s\"", model_dir.c_str());
    return 3;
  }
  
  string abspath(c_abspath);
  free(c_abspath);

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>(topic_name.c_str(),
    10);
 
  sleep(1);

  if(ObjectModel::checkDirectory(abspath))
  {
    ObjectModel model(abspath, false);
    
    std_msgs::String msg;
    msg.data = abspath;

    ROS_INFO("Sending \"%s\" by topic %s", abspath.c_str(), topic_name.c_str());

    pub.publish(msg);

    ros::spinOnce();
    
    return 0;  
  }
  else
  {
    ROS_ERROR("The directory \"%s\" is not a valid object model. Nothing done.",
      model_dir.c_str());
    return 4;
  }
}

// ---------------------------------------------------------------------------


