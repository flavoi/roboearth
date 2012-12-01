/** \file ObjectDetectorMethod.cpp
 * \brief Interface for object detection algorithms
 *
 * Abstract class to implement different object detection algorithms
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


#include <opencv/cv.h>
#include <vector>
#include "ObjectDetectorMethod.h"
#include "ObjectModel.h"
#include "CameraBridge.h"

#include "DUtilsCV.h"
#include "DVision.h"
#include "epnp.h"

using namespace std;

typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;

// ---------------------------------------------------------------------------

void ObjectDetectorMethod::calculatePose(const ObjectModel::Face &face, 
  const std::vector<int> &face_indices, const DVision::SurfSet &scene_surfset, 
  const std::vector<int> &scene_indices, const CameraBridge &camera,
  cv::Mat &cTo) const
{
  const unsigned int N = face_indices.size();
  const cv::Mat &A = camera.GetIntrinsicParameters();
  
  epnp PnP;
  PnP.set_maximum_number_of_correspondences(N);
  PnP.set_internal_parameters(
    A.at<float>(0, 2), A.at<float>(1, 2), A.at<float>(0, 0), A.at<float>(1, 1));

  PnP.reset_correspondences();

  for(unsigned int i = 0; i < N; ++i)
  {
    const PLYPoint &p3d = face.plypoints[ face_indices[i] ];
    const cv::KeyPoint &kp = scene_surfset.keys[ scene_indices[i] ];

    PnP.add_correspondence(p3d.x, p3d.y, p3d.z, kp.pt.x, kp.pt.y);
  }

  double cRo_r[3][3], cto_r[3];
  PnP.compute_pose(cRo_r, cto_r);
  
  cv::Mat cRo(3, 3, CV_64F, cRo_r);
  cv::Mat cto(3, 1, CV_64F, cto_r);
  
  cTo = DUtilsCV::Transformations::composeRt(cRo, cto);

  /* // CV version
  const unsigned int N = face_indices.size();
  cv::Mat objectPoints(N, 3, CV_32F);
  cv::Mat scenePoints(N, 2, CV_32F);

  for(unsigned int i = 0; i < N; ++i)
  {
    const PLYPoint &p3d = face.plypoints[ face_indices[i] ];
    const cv::KeyPoint &k2d = scene_surfset.keys[ scene_indices[i] ];
    
    objectPoints.at<float>(i, 0) = p3d.x;
    objectPoints.at<float>(i, 1) = p3d.y;
    objectPoints.at<float>(i, 2) = p3d.z;
    
    scenePoints.at<float>(i, 0) = k2d.pt.x;
    scenePoints.at<float>(i, 1) = k2d.pt.y;
  }
  
  cv::Mat cRo, cto;
  cv::solvePnP(objectPoints, scenePoints, camera.GetIntrinsicParameters(),
    cv::Mat::zeros(4,1, CV_32F), cRo, cto); // no distortion
    
  cTo = DUtilsCV::Transformations::composeRt(cRo, cto);
  */
}

// ---------------------------------------------------------------------------

void ObjectDetectorMethod::convertPose(const cv::Mat &T, 
  geometry_msgs::Pose &pose) const
{
  if(T.type() == CV_64F)
  {
    pose.position.x = T.at<double>(0, 3) / T.at<double>(3, 3);
    pose.position.y = T.at<double>(1, 3) / T.at<double>(3, 3);
    pose.position.z = T.at<double>(2, 3) / T.at<double>(3, 3);
  }
  else
  {
    pose.position.x = T.at<float>(0, 3) / T.at<float>(3, 3);
    pose.position.y = T.at<float>(1, 3) / T.at<float>(3, 3);
    pose.position.z = T.at<float>(2, 3) / T.at<float>(3, 3);
  }
  
  // rot to quaternion (matlab)
  geometry_msgs::Quaternion &q = pose.orientation;

  double a[3][3]; // a contains the traspose of the rotation of T...
  
  if(T.type() == CV_64F)
  {
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        a[i][j] = T.at<double>(j,i);
  }
  else
  {
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        a[i][j] = T.at<float>(j,i);
  }
  
  double trace= a[0][0] + a[1][1] + a[2][2];

  if (trace > 0){
    double sqtrp1 = sqrt( trace + 1.0 );

    q.w = 0.5*sqtrp1;
    q.x = (a[1][2] - a[2][1])/(2.0*sqtrp1);
    q.y = (a[2][0] - a[0][2])/(2.0*sqtrp1);
    q.z = (a[0][1] - a[1][0])/(2.0*sqtrp1);
  }else{
    if ( (a[1][1] > a[0][0]) && (a[1][1] > a[2][2])){
      double sqdip1 = sqrt(a[1][1] - a[0][0] - a[2][2] + 1.0 );

      q.y = 0.5*sqdip1;

      if ( sqdip1 != 0 ) sqdip1 = 0.5/sqdip1;

      q.w = (a[2][0] - a[0][2])*sqdip1;
      q.x = (a[0][1] + a[1][0])*sqdip1;
      q.z = (a[1][2] + a[2][1])*sqdip1;
    }else if (a[2][2] > a[0][0]){
      double sqdip1 = sqrt(a[2][2] - a[0][0] - a[1][1] + 1.0 );

      q.z = 0.5*sqdip1;

      if ( sqdip1 != 0 ) sqdip1 = 0.5/sqdip1;

      q.w = (a[0][1] - a[1][0])*sqdip1;
      q.x = (a[2][0] + a[0][2])*sqdip1;
      q.y = (a[1][2] + a[2][1])*sqdip1;
    }else{
      double sqdip1 = sqrt(a[0][0] - a[1][1] - a[2][2] + 1.0 );

      q.x = 0.5*sqdip1;

      if ( sqdip1 != 0 ) sqdip1 = 0.5/sqdip1;

      q.w = (a[1][2] - a[2][1])*sqdip1;
      q.y = (a[0][1] + a[1][0])*sqdip1;
      q.z = (a[2][0] + a[0][2])*sqdip1;
    }
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorMethod::convert3DPoints(const ObjectModel::Face &face, 
  const std::vector<int> &indices, 
  const cv::Mat &cTo, 
  std::vector<geometry_msgs::Point> &points3d) const
{
  assert(cTo.type() == CV_64F);
  
  cv::Mat oP(4, indices.size(), CV_64F);

  for(unsigned int i = 0; i < indices.size(); ++i)
  {
    const PLYPoint &ply = face.plypoints[indices[i]];
    oP.at<double>(0, i) = ply.x;
    oP.at<double>(1, i) = ply.y;
    oP.at<double>(2, i) = ply.z;
    oP.at<double>(3, i) = 1;
  }
  
  cv::Mat cP = cTo * oP;
  
  points3d.resize(indices.size());
  for(unsigned int i = 0; i < indices.size(); ++i)
  {
    points3d[i].x = cP.at<double>(0, i) / cP.at<double>(3, i);
    points3d[i].y = cP.at<double>(1, i) / cP.at<double>(3, i);
    points3d[i].z = cP.at<double>(2, i) / cP.at<double>(3, i);
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorMethod::convert3DPoints(const ObjectModel::Face &face,
  const cv::Mat &cTo,
  std::vector<geometry_msgs::Point> &points3d) const
{
  assert(cTo.type() == CV_64F);
  
  const unsigned int N = face.plypoints.size();
  
  cv::Mat oP(4, N, CV_64F);

  for(unsigned int i = 0; i < N; ++i)
  {
    const PLYPoint &ply = face.plypoints[i];
    oP.at<double>(0, i) = ply.x;
    oP.at<double>(1, i) = ply.y;
    oP.at<double>(2, i) = ply.z;
    oP.at<double>(3, i) = 1;
  }
  
  cv::Mat cP = cTo * oP;
  
  points3d.resize(N);
  for(unsigned int i = 0; i < N; ++i)
  {
    points3d[i].x = cP.at<double>(0, i) / cP.at<double>(3, i);
    points3d[i].y = cP.at<double>(1, i) / cP.at<double>(3, i);
    points3d[i].z = cP.at<double>(2, i) / cP.at<double>(3, i);
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorMethod::get3DModelPoints(const ObjectModel::Face &face, 
    std::vector<geometry_msgs::Point> &points3d) const
{
  const unsigned int N = face.plypoints.size();
  points3d.resize(N);

  for(unsigned int i = 0; i < N; ++i)
  {
    const PLYPoint &ply = face.plypoints[i];
    
    points3d[i].x = ply.x;
    points3d[i].y = ply.y;
    points3d[i].z = ply.z;
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorMethod::project2DPoints(const ObjectModel::Face &face, 
  const cv::Mat &cTo, const CameraBridge &camera,
  std::vector<re_msgs::Pixel> &points2d) const
{
  const unsigned int N = face.plypoints.size();
  
  cv::Mat oP(4, N, CV_64F);
  
  for(unsigned int i = 0; i < N; ++i)
  {
    const PLYPoint &ply = face.plypoints[i];
    oP.at<double>(0, i) = ply.x;
    oP.at<double>(1, i) = ply.y;
    oP.at<double>(2, i) = ply.z;
    oP.at<double>(3, i) = 1;
  }
  
  cv::Mat cP = cTo * oP;
  
  cv::Mat oP_4x3(oP.cols, 3, CV_32F);
  for(int c = 0; c < oP.cols; ++c)
    for(int r = 0; r < 3; ++r)
      oP_4x3.at<float>(c, r) = oP.at<double>(r, c) / oP.at<double>(3, c);
  
  cv::Mat cRo;
  cv::Mat cto(3, 1, CV_64F);
  DUtilsCV::Transformations::decomposeRt(cTo, cRo, cto);

  vector<cv::Point2f> cam_pixels;
  cv::projectPoints(oP_4x3, cRo, cto, camera.GetIntrinsicParameters(), 
    cv::Mat::zeros(4,1,CV_32F), cam_pixels);
  
  // copy back
  points2d.resize(N);
  for(unsigned int i = 0; i < N; ++i)
  {
    points2d[i].x = cam_pixels[i].x;
    points2d[i].y = cam_pixels[i].y;
  }
  
}

// ---------------------------------------------------------------------------

void ObjectDetectorMethod::changeOrientation(const cv::Mat &cTo, cv::Mat &rTo) 
  const
{
  // rTo = rotz(-pi/2) * rotx(-pi/2) * cTo
  // --> rTo[0,:] =  cTo[2,:]
  //     rTo[1,:] = -cTo[0,:]
  //     rTo[2,:] = -cTo[1,:]
  //     rTo[3,:] =  cTo[3,:]
  if(cTo.type() == CV_32F)
  {
    rTo = (cv::Mat_<float>(4, 4) <<
      cTo.at<float>(2,0), cTo.at<float>(2,1), cTo.at<float>(2,2), cTo.at<float>(2,3),
      -cTo.at<float>(0,0), -cTo.at<float>(0,1), -cTo.at<float>(0,2), -cTo.at<float>(0,3),
      -cTo.at<float>(1,0), -cTo.at<float>(1,1), -cTo.at<float>(1,2), -cTo.at<float>(1,3),
      cTo.at<float>(3,0), cTo.at<float>(3,1), cTo.at<float>(3,2), cTo.at<float>(3,3));
  }
  else
  {
    rTo = (cv::Mat_<double>(4, 4) <<
      cTo.at<double>(2,0), cTo.at<double>(2,1), cTo.at<double>(2,2), cTo.at<double>(2,3),
     -cTo.at<double>(0,0), -cTo.at<double>(0,1), -cTo.at<double>(0,2), -cTo.at<double>(0,3),
     -cTo.at<double>(1,0), -cTo.at<double>(1,1), -cTo.at<double>(1,2), -cTo.at<double>(1,3),
      cTo.at<double>(3,0), cTo.at<double>(3,1), cTo.at<double>(3,2), cTo.at<double>(3,3));
  }
}

// ---------------------------------------------------------------------------


