/** \file testModel.cpp
 * \brief Shows model data
 *
 * TestModel creates images of the faces of a 3D model with the SURF features
 * and the object reference
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

// Usage: testModel <model_dir>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../ObjectModel.h"

#include "ros/ros.h"
#include "DUtils.h"
#include "DUtilsCV.h"

using namespace std;

// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "testModel");
  
  if(argc < 1)
  {
    cout << "Usage: " << argv[0] << " <model dir>" << endl;
    return 0;
  }

  string model_dir = argv[1];
  
  cout << "-- Creating test directory" << endl;
  
  string test_dir = model_dir + "/test";
  DUtils::FileFunctions::MkDir(test_dir.c_str());
  
  cout << "-- Loading model" << endl;
  
  ObjectModel model;
  model.loadDirectory(model_dir, true);

  vector<ObjectModel::Face>::const_iterator fit;
  for(fit = model.Faces.begin(); fit != model.Faces.end(); ++fit)
  {
    const int idx = fit - model.Faces.begin();
    
    cv::Mat cTo = DUtilsCV::Transformations::composeRt(fit->cRo, fit->cto);
    
    cout << "-- Copying image " << idx << endl;
    cv::Mat image, aux;
    if(fit->image.channels() == 3)
    {
      cv::cvtColor(fit->image, aux, CV_RGB2GRAY);
      cv::cvtColor(aux, image, CV_GRAY2RGB);
    }else
    {
      cv::cvtColor(fit->image, image, CV_GRAY2RGB);
    }

    cout << "-- Drawing image " << idx << endl;

    DUtilsCV::Drawing::drawKeyPoints(image, fit->surf.keys);
    model.getVisualizationModel().draw(image, cTo, fit->A);
    
    char buffer[512];
    sprintf(buffer, "%s/test_%03d.png", test_dir.c_str(), idx);
    cv::imwrite(buffer, image);
  }
  
  cout << "Done. Results saved in " << test_dir << endl;

}

// ---------------------------------------------------------------------------


