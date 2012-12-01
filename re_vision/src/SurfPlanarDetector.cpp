/** \file SurfPlanarDetector.cpp
 * \brief Detection algorithm for planar objects
 *
 * ObjectDetectorMethod for planar textured objects
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

#include "SurfPlanarDetector.h"
#include "PlanarVisualizationModel.h"
#include "ObjectModel.h"
#include "ObjectDetectorMethod.h"
#include "re_msgs/DetectedObject.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

#include "debug.h"

#include <vector>
#include <iomanip>

typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;
typedef DVision::SurfSet SurfSet;
using namespace std;

// ---------------------------------------------------------------------------

/// IF_DEBUG_MODE(X) executes code X only if PLANAR_DEBUG_ENABLED is 1
/// and if the debug_mode flag was given to this instance
/// IF_DEBUG_OR_VISUALIZATION_MODE(data, X) executes code X only if
/// visualization in data is enabled or if we are in debug mode

#ifdef IF_DEBUG_MODE
  #undef IF_DEBUG_MODE
#endif

#if PLANAR_DEBUG_ENABLED
  #define IF_DEBUG_MODE(X) if(debugMode()){ X; }
  #define IF_DEBUG_OR_VISUALIZATION_MODE(X) \
    if(debugMode() || visualizationMode()){ X; }
#else
  #define IF_DEBUG_MODE(X) if(0){}
  // this definition enables the use of "else"
  #define IF_DEBUG_OR_VISUALIZATION_MODE(X) \
    if(visualizationMode()){ X; }
#endif

// ---------------------------------------------------------------------------

void SurfPlanarDetector::detect(ObjectDetectorMethod::DetectionData &data, 
  ObjectModel &model,
  re_msgs::DetectedObject &detection)
{

  #if PLANAR_DEBUG_ENABLED
    // sets prefix of debug files
    {
      stringstream ss;
      ss << m_debug_dir << "/"
        << setw(2) << setfill('0') << m_detect_counter << "_planar_";
      m_debug_prefix = ss.str();      
    }
    std::string original_debug_prefix = m_debug_prefix;
    m_detect_counter++;
  #endif
  
  if(!data.surfdata.provided)
  {
    // calculate surfs
    data.surfdata.surfset.Extract(data.img, 400, false);
    data.surfdata.provided = true;
  }

  IF_DEBUG_OR_VISUALIZATION_MODE
  (
    cv::Mat img = data.img.clone();
    DUtilsCV::Drawing::drawKeyPoints(img, data.surfdata.surfset.keys);
    
    IF_DEBUG_MODE
    (
      std::string filename = m_debug_prefix + "0_scene_keypoints.png";
      cv::imwrite(filename, img);
    )
    
    if(visualizationMode()) visualize(img, "Searching...");
  )

  const SurfSet& scene_surfset = data.surfdata.surfset;
  vector<int> face_indices;
  vector<vector<int> > model_indices;
  vector<vector<int> > scene_indices;
  vector<vector<float> > distances;
 
  model.detectFaces(scene_surfset, face_indices, model_indices, 
    scene_indices, distances, 1, 1, 0.6);
    
  bool found = false;
  if(!face_indices.empty())
  {
    for(unsigned int i = 0; i < face_indices.size(); ++i)
    {
      int idx = face_indices[i];
                  
      IF_DEBUG_MODE
      (
        stringstream ss;
        ss << original_debug_prefix << "1_it_" << i << "_face_" << idx << "_";
        m_debug_prefix = ss.str();
        std::string filename = m_debug_prefix + "0_match.png";
        
        DUtilsCV::Drawing::saveCorrespondenceImage(filename, data.img, 
          model.Faces[idx].image, scene_surfset.keys, 
          model.Faces[idx].surf.keys,
          scene_indices[i], model_indices[i]);
      )
      
      found = detectWithHomography(scene_surfset, model, model.Faces[idx], 
        model_indices[i], scene_indices[i], distances[i],
        data, 3., false, detection); 
      
      if(found) break;
    }
  }
  
  if(!found)
  {
    detection.points2d.clear();
    detection.points3d.clear();
    detection.points3d_model.clear();
  }
}

// ---------------------------------------------------------------------------

bool SurfPlanarDetector::alignedPoints(const ObjectModel::Face &face, 
    const std::vector<int> indices) const
{
  if(indices.size() < 3) return true;
  
  const float sq_threshold = 0.15 * 0.15;
  
  cv::Mat P(indices.size(), 3, CV_32F);
  float *p = P.ptr<float>();
  for(unsigned int i = 0; i < indices.size(); ++i, p += 3)
  {
    const PLYPoint &ply = face.plypoints[indices[i]];
    
    p[0] = ply.x;
    p[1] = ply.y;
    p[2] = ply.z;
  }
  
  cv::Mat Q, mean;
  cv::calcCovarMatrix(P, Q, mean, CV_COVAR_ROWS | CV_COVAR_NORMAL);
  
  float d[3] = {
    Q.at<float>(0,0), Q.at<float>(1,1), Q.at<float>(2,2) };
  
  sort(d, d+3); // ascending
  
  return (d[1] / d[2] < sq_threshold);
}

// ---------------------------------------------------------------------------

bool SurfPlanarDetector::detectWithHomography(const SurfSet &scene_surfset,
  ObjectModel &model, ObjectModel::Face &face, 
  const std::vector<int> &model_indices,
  const std::vector<int> &scene_indices, const std::vector<float> &distances,
  const ObjectDetectorMethod::DetectionData &data,
  const double maxReprojectionError, bool doReChecking,
  re_msgs::DetectedObject &detection) 
{
  const int MIN_POINTS = 7;
  const int N = model_indices.size();

  if(N < MIN_POINTS) return false;  
  
  // homography 
  cv::Mat model_points(N, 2, CV_32F);
  cv::Mat scene_points(N, 2, CV_32F);

  for(int i = 0; i < N; ++i)
  {
    const cv::KeyPoint &model_kp = face.surf.keys[model_indices[i]];
    const cv::KeyPoint &scene_kp = scene_surfset.keys[scene_indices[i]];

    model_points.at<float>(i, 0) = model_kp.pt.x;    
    model_points.at<float>(i, 1) = model_kp.pt.y;
    
    scene_points.at<float>(i, 0) = scene_kp.pt.x;    
    scene_points.at<float>(i, 1) = scene_kp.pt.y;
  }

  vector<unsigned char> status;
  cv::Mat sHm = cv::findHomography(model_points, scene_points, status, 
    CV_RANSAC, maxReprojectionError);
  
  cv::Mat _image_to_show;
  IF_DEBUG_OR_VISUALIZATION_MODE
  (
    vector<int> debug_scene_indices;
    vector<int> debug_model_indices;
    debug_scene_indices.reserve(status.size());
    debug_model_indices.reserve(status.size());
    
    for(unsigned int i = 0; i < status.size(); ++i)
    {
      if(status[i] != 0)
      {
        debug_scene_indices.push_back( scene_indices[i] );
        debug_model_indices.push_back( model_indices[i] );
      }
    }
    
    DUtilsCV::Drawing::drawCorrespondences(_image_to_show, data.img,
      face.image, scene_surfset.keys, face.surf.keys,
      debug_scene_indices, debug_model_indices);  
    
    IF_DEBUG_MODE
    ( 
      std::string filename;
      if(doReChecking)
        filename = m_debug_prefix + "1_0_first_homography.png";
      else
        filename = m_debug_prefix + "1_1_re_homography.png";
        
      cv::imwrite(filename, _image_to_show); 
    )
  )
  
  // check if there are enough inliers
  if(!sHm.empty())
  {
    vector<int> inlier_model_indices, inlier_scene_indices;
    vector<float> inlier_distances;
    
    inlier_scene_indices.reserve(status.size());
    inlier_model_indices.reserve(status.size());
    inlier_distances.reserve(status.size());
    
    for(unsigned int i = 0; i < status.size(); ++i)
    {
      if(status[i] != 0)
      { 
        inlier_model_indices.push_back( model_indices[i] );
        inlier_scene_indices.push_back( scene_indices[i] );
        inlier_distances.push_back( distances[i] );
      }
    }
    
    if(!alignedPoints(face, inlier_model_indices))
    {
    
      int inliers = (int)inlier_model_indices.size();
      
      // find a better set of matches?
      if(doReChecking && inliers >= MIN_POINTS)
      {
        bool found = detectWithHomography(scene_surfset, model, face, 
          inlier_model_indices, inlier_scene_indices, inlier_distances, 
          data, 1, false, detection);
        
        if(found) return true;
      }
      
      if(inliers >= MIN_POINTS)
      { 
        // found!
        //if(visualizationMode())
        //{
        //  visualize(_image_to_show);
        //}
        
        // calculate pose
        cv::Mat cTo;
        calculatePose(face, inlier_model_indices, scene_surfset, 
          inlier_scene_indices, data.params.camera, cTo);
        
        //assert(cTo.type() == CV_64F);
        
        if(cTo.at<double>(2,3) / cTo.at<double>(3,3) <= 0)
        {
          ROS_DEBUG("cTo behind the camera, rejecting...");
        }
        else
        {
          // obj is in front of the camera

          IF_DEBUG_OR_VISUALIZATION_MODE
          (
            cv::Mat img;
            if(img.channels() == 3)
              img = data.img.clone();
            else
              cv::cvtColor(data.img, img, CV_GRAY2BGR);
            
            model.getVisualizationModel().draw(img, cTo, 
              data.params.camera.GetIntrinsicParameters());
            
            IF_DEBUG_MODE
            ( 
              string filename = m_debug_prefix + "2_pose.png";
              cv::imwrite(filename, img);
            ) 
            
            if(visualizationMode()) visualize(img, 
              model.Name + " located!", true); 
          )
          
          // convert pose form camera orientation to robot orientation
          cv::Mat cTo_robot;
          changeOrientation(cTo, cTo_robot);

          // fill DetectedObject data
          convertPose(cTo_robot, detection.pose);
          // convert3DPoints(face, inlier_model_indices, cTo_robot, detection.points3d);
          
          // get all the points from the model and their corresponding ones 
          // in the scene
          convert3DPoints(face, cTo_robot, detection.points3d);
          project2DPoints(face, cTo, data.params.camera, detection.points2d);
          get3DModelPoints(face, detection.points3d_model);
          
          IF_DEBUG_MODE
          (
            //DUtilsCV::IO::print(cTo_robot, "--> rTo");
            fstream f((m_debug_prefix + "rTo.txt").c_str(), ios::out);
            DUtilsCV::IO::print(cTo_robot, "rTo", f);
            f.close();
            //debug::saveTscene(cTo_robot, detection.points3d, 
            //  m_debug_prefix + "scene.wrl");
          )
          
          /*
          detection.points2d.resize(0); 
          detection.points2d.reserve(inliers);
          detection.points3d_model.resize(0); 
          detection.points3d_model.reserve(inliers);
          
          for(unsigned int i = 0; i < inlier_model_indices.size(); ++i)
          {
            const PLYPoint &ply = face.plypoints[inlier_model_indices[i]];
            const cv::KeyPoint &scene_kp = scene_surfset.keys[inlier_scene_indices[i]];
            
            re_msgs::Pixel p2d;
            p2d.x = cvRound(scene_kp.pt.x);
            p2d.y = cvRound(scene_kp.pt.y);
            detection.points2d.push_back(p2d);
            
            geometry_msgs::Point p3d;
            p3d.x = ply.x;
            p3d.y = ply.y;
            p3d.z = ply.z;
            detection.points3d_model.push_back(p3d);
          }
          */
          
          return true;
        } // if Z positive
      } // if points not aligned
    } // if inliers >= min_inliers
  } // if ! sHm empty
  
  return false;
}

// ---------------------------------------------------------------------------

