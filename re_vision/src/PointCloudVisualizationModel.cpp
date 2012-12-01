/** \file PointCloudVisualizationModel.cpp
 * \brief Visualization model of a 3d object composed of 3d points
 *
 * Class to store a 3d object visualization model composed of 3d points
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

#include "VisualizationModel.h"
#include "PointCloudVisualizationModel.h"
#include <cv.h>
#include <string>
#include <vector>

#include "DVision.h"

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;

using namespace std;

// ---------------------------------------------------------------------------

PointCloudVisualizationModel::PointCloudVisualizationModel
  (const std::string &filename)
{
  PLYFile::readFile(filename, m_plypoints);
  convertPLY2Mat();
  calculateDimensions();
}

// ---------------------------------------------------------------------------

void PointCloudVisualizationModel::convertPLY2Mat()
{
  m_oP.create(4, m_plypoints.size(), CV_64F);
  
  double *px = m_oP.ptr<double>(0);
  double *py = m_oP.ptr<double>(1);
  double *pz = m_oP.ptr<double>(2);
  double *ps = m_oP.ptr<double>(3);
  
  vector<PLYPoint>::const_iterator pit;
  for(pit = m_plypoints.begin(); pit != m_plypoints.end(); 
    ++pit, ++px, ++py, ++pz, ++ps)
  {
    *px = pit->x;
    *py = pit->y;
    *pz = pit->z;
    *ps = 1;
  }
}

// ---------------------------------------------------------------------------

void PointCloudVisualizationModel::calculateDimensions()
{
  if(m_plypoints.empty())
  {
    m_dimx = m_dimy = m_dimz = 0;
  }else
  {
    const double *px = m_oP.ptr<double>(0);
    const double *py = m_oP.ptr<double>(1);
    const double *pz = m_oP.ptr<double>(2);
    const int N = m_oP.cols;
    
    double minx = *std::min_element(px, px + N);
    double miny = *std::min_element(py, py + N);
    double minz = *std::min_element(pz, pz + N);
    
    double maxx = *std::max_element(px, px + N);
    double maxy = *std::max_element(py, py + N);
    double maxz = *std::max_element(pz, pz + N);
    
    m_dimx = maxx - minx;
    m_dimy = maxy - miny;
    m_dimz = maxz - minz;
  }
}

// ---------------------------------------------------------------------------

void PointCloudVisualizationModel::draw(cv::Mat &img, const cv::Mat &_cTo,
  const cv::Mat &A, const cv::Mat &K) const
{  
  cv::Mat k;
  if(K.empty())
    k = cv::Mat::zeros(4, 1, A.type());
  else
    k = K;
  
  cv::Mat cTo;
  if(_cTo.type() == CV_64F)
    cTo = _cTo;
  else
    _cTo.convertTo(cTo, CV_64F);
  
  cv::Mat cP = cTo * m_oP;
  
  vector<pair<double, int> > depths; // <z, point index>
  depths.reserve(cP.cols);
  
  if(cTo.at<double>(3,3) != 1)
  {
    // normalize 
    double *px = cP.ptr<double>(0);
    double *py = cP.ptr<double>(1);
    double *pz = cP.ptr<double>(2);
    double *ps = cP.ptr<double>(3);
    for(int i = 0; i < cP.cols; ++i, ++px, ++py, ++pz, ++ps)
    {
      *px /= *ps;
      *py /= *ps;
      *pz /= *ps;
      *ps = 1;
      
      depths.push_back(make_pair(*pz, i));
    }
  }
  
  if(depths.empty())
  {
    double *pz = cP.ptr<double>(2);
    for(int i = 0; i < cP.cols; ++i, ++pz)
    {
      depths.push_back(make_pair(*pz, i));
    }
  }
  
  // sort in ascending z
  sort(depths.begin(), depths.end());
  
  // project now to 2d
  vector<cv::Point2f> points2d;
  
  cv::Mat cP_Nx3;
  cP.convertTo(cP_Nx3, CV_32F);
  cP_Nx3 = cP_Nx3.rowRange(0,3).t();
  
  cv::projectPoints(cP_Nx3, 
    cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F),
    A, k, points2d);
  
  // draw the further points first
  bool frame_painted = false;
  vector<pair<double, int> >::const_reverse_iterator pit;
  for(pit = depths.rbegin(); pit != depths.rend(); ++pit)
  {
    int pidx = pit->second;
    const PLYPoint& pp = m_plypoints[pidx];
    const cv::Point2f p2 = points2d[pidx];
    
    if(!frame_painted && pp.z > 0)
    {
      // time to paint the reference system
      frame_painted = true;
      
      // draw a reference system
      float length;
      float d[3] = {m_dimx, m_dimy, m_dimz};
      sort(d, d+3); // ascending

      if(d[0] < d[2]/3.f)
      {
        // thin object
        length = 0.5f * d[1];
      }
      else
      {
        // almost a cube
        length = 0.5f * d[0];
      }
    }
    
    CvScalar c = cvScalar(pp.b, pp.g, pp.r);
    cv::circle(img, cvPoint(p2.x, p2.y), 0, c);
    cv::circle(img, cvPoint(p2.x, p2.y), 1, c);
  }

}

// ---------------------------------------------------------------------------


