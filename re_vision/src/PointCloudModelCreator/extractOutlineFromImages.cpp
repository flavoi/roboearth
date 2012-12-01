/** \file extractOutlineFromImages.cpp
 * \brief Extracts the outline of a set of images
 *
 * Standalone application for extracting the outline of segmented images
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

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <vector>
#include <fstream>

#include "DUtils.h"
#include "DUtilsCV.h"

using namespace std;

// ----------------------------------------------------------------------------

void extractMask(const cv::Mat &image, cv::Mat &mask);
void getSegmentedFilenames(const std::string &list_file, 
  std::vector<std::string> &normal_files,
  std::vector<std::string> &segmented_files,
  const std::string &segmented_model_dir);
void generateMasks(const std::vector<std::string> &normal_files,
  const std::vector<std::string> &segmented_files,
  const std::string &out_dir);

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{

  if(argc < 4)
  {
    cout << "Usage: " << argv[0] << " <list.rd.out file> <out dir> <segmented dir>" << endl;
    return 1;
  }

  std::string list_file = argv[1];
  std::string out_dir = argv[2];
  std::string segmented_model_dir = argv[3];
  
  vector<string> segmented_files, normal_files;
  
  try
  {
    cout << "-- Getting filenames..." << endl;
    getSegmentedFilenames(list_file, normal_files, segmented_files, segmented_model_dir);

    cout << "-- Generating masks..." << endl;
    generateMasks(normal_files, segmented_files, out_dir);
  }
  catch(std::string ex)
  {
    cout << ex << endl;
  }

}

// ----------------------------------------------------------------------------

void getSegmentedFilenames(const std::string &list_file, 
  std::vector<std::string> &normal_files,
  std::vector<std::string> &segmented_files,
  const std::string &segmented_model_dir)
{
  // assumes segmented files are in a directory with the same
  // name as the original, but with the suffix "SinFondo"
  
  segmented_files.clear();
  normal_files.clear();
  
  fstream f(list_file.c_str(), ios::in);
  if(!f.is_open())
  {
    throw(string("Could not open file ") + list_file);
    return;
  }
  
  string line;
  while(!f.eof())
  {
    getline(f, line);
    if(!f.eof() && !line.empty())
    {
      string path, name, ext;
      DUtils::FileFunctions::FileParts(line, path, name, ext);
      
      normal_files.push_back(line);

      segmented_files.push_back(
        segmented_model_dir + "/" + name + "." + ext);
      
//      segmented_files.push_back(
//        path + "SinFondo/" + name + "." + ext);
    }
  }
  
  f.close();
}

// ----------------------------------------------------------------------------

void generateMasks(const std::vector<std::string> &normal_files,
  const std::vector<std::string> &segmented_files,
  const std::string &out_dir)
{
  vector<string>::const_iterator nit, sit;
  nit = normal_files.begin();
  sit = segmented_files.begin();

  for(; nit != normal_files.end(); ++nit, ++sit)
  {
    int idx = nit - normal_files.begin();
    
    //string path, filename, ext;
    //DUtils::FileFunctions::FileParts(*nit, path, filename, ext);
    
    cv::Mat im = cv::imread(*sit, 0);
    
    if(im.empty())
    {
      throw string("Could not open image " + *sit);
    }
    
    cv::Mat mask;
    extractMask(im, mask);

    char buffer[512];    
    sprintf(buffer, "%s/im%02d_mask.png", out_dir.c_str(), idx);
    cv::imwrite(buffer, mask);
  }

}

// ----------------------------------------------------------------------------

struct xy { 
  int x, y; 
  xy(int _x, int _y): x(_x), y(_y) {} 
};

void extractMask(const cv::Mat &image, cv::Mat &mask)
{
  mask.create(image.rows, image.cols, CV_8U);
  mask = 255;
  
  // set 1 if foreground pixels
  // assume pixel 0,0 is always background and that background is connected
  
  cv::Mat visited(image.rows, image.cols, CV_8U);
  visited = cv::Scalar(0);
  
  vector<xy> queue;
  queue.push_back(xy(0,0));
  visited.at<unsigned char>(0,0) = 1;

  // 4-connectivity
  const int N = 4;
  const int dx[N] = {-1, 0, +1, 0};
  const int dy[N] = {0, -1, 0, +1};

  while(!queue.empty())
  {
    xy p = queue.back();
    queue.pop_back();

    for(int i = 0; i < N; ++i)
    {
      int x = p.x + dx[i];
      int y = p.y + dy[i];
      
      if(x >= 0 && x < image.cols && y >= 0 && y < image.rows &&
        visited.at<unsigned char>(y,x) == 0)
      {
        visited.at<unsigned char>(y,x) = 1;
        
        // original images are jpg, so there are black pixels not so black
        if(image.at<unsigned char>(y,x) < 5)
        {
          queue.push_back(xy(x,y));
          mask.at<unsigned char>(y,x) = 0;
        }
      } // if not visited
    } // for each neighbor
  } // for each item in the queue
  
}

// ----------------------------------------------------------------------------

