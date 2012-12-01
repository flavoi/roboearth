/** \file Surf3DDetector.cpp
 * \brief Detection algorithm for 3D objects
 *
 * ObjectDetectorMethod for 3D textured objects using SURF
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

#include "Surf3DDetector.h"
#include "ObjectModel.h"
#include "ObjectDetectorMethod.h"
#include "re_msgs/DetectedObject.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"
#include "epnp.h"

#include <vector>
#include <iomanip>

typedef DUtils::Random Random;
typedef DVision::SurfSet SurfSet;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;
using namespace std;

// ---------------------------------------------------------------------------

/// IF_DEBUG_MODE(X) executes code X only if VOLUME_DEBUG_ENABLED is 1
/// and if the debug_mode flag was given to this instance

#ifdef IF_DEBUG_MODE
  #undef IF_DEBUG_MODE
#endif

#if VOLUME_DEBUG_ENABLED
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

void Surf3DDetector::detect(ObjectDetectorMethod::DetectionData &data, 
  ObjectModel &model,
  re_msgs::DetectedObject &detection)
{  
  #if VOLUME_DEBUG_ENABLED
    // sets prefix of debug files
    {
      stringstream ss;
      ss << m_debug_dir << "/"
        << setw(2) << setfill('0') << m_detect_counter << "_3d_";
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
      
      found = detectWithPnP(scene_surfset, model, model.Faces[idx], 
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

bool Surf3DDetector::detectWithPnP(const SurfSet &scene_surfset,
    ObjectModel &model,
    ObjectModel::Face &face, const std::vector<int> &model_indices,
    const std::vector<int> &scene_indices, const std::vector<float> &distances,
    const ObjectDetectorMethod::DetectionData &data,
    const double maxReprojectionError, bool doReChecking,
    re_msgs::DetectedObject &detection)
{
  const int MIN_POINTS = 5;
  
  if((int)model_indices.size() < MIN_POINTS) return false;

  const int N = 5; // model points
  int max_its = 200; // max iterations
  
  const cv::Mat &A = data.params.camera.GetIntrinsicParameters();
  const cv::Mat &K = data.params.camera.GetDistortionParameters();
    
  //assert(A.type() == CV_32F);
  
  epnp PnP;
  PnP.set_maximum_number_of_correspondences(N);
  PnP.set_internal_parameters(
    A.at<float>(0, 2), A.at<float>(1, 2), A.at<float>(0, 0), A.at<float>(1, 1));
  
  // stores the correspondence points of the model in a matrix
  cv::Mat oP_Nx3 = cv::Mat(model_indices.size(), 3, CV_32F);
  
  // stores the correspondence points of the scene in a matrix
  cv::Mat scene_2d_points_Nx2(model_indices.size(), 2, CV_32F);
  
  // stores the projected points of the model
  vector<cv::Point2f> vec_object_2d_points;
  vec_object_2d_points.reserve(model_indices.size());
  
  // for operating with projected points
  cv::Mat projected_distance_xy_Nx2(model_indices.size(), 2, CV_32F);
  cv::Mat projected_sq_distance_Nx1(model_indices.size(), 1, CV_32F);
  cv::Mat inlier_status_Nx1(model_indices.size(), 1, CV_8UC1);
  static vector<unsigned int> i_inliers;
  i_inliers.reserve(model_indices.size());
  
  float *p_oP = oP_Nx3.ptr<float>();
  float *p_scene_2d = scene_2d_points_Nx2.ptr<float>();
  for(unsigned int i = 0; i < model_indices.size(); 
    ++i, p_oP += 3, p_scene_2d += 2)
  {
    const PLYPoint &p3d = face.plypoints[ model_indices[i] ];
    const cv::KeyPoint kp = scene_surfset.keys[ scene_indices[i] ];
    
    p_oP[0] = p3d.x;
    p_oP[1] = p3d.y;
    p_oP[2] = p3d.z;
    
    p_scene_2d[0] = kp.pt.x;
    p_scene_2d[1] = kp.pt.y;
  }
  
  // stores the candidate transformation to the object
  double cRo_r[3][3], cto_r[3];
  cv::Mat cRo(3, 3, CV_64F, cRo_r);
  cv::Mat cto(3, 1, CV_64F, cto_r);
    
  Random::SeedRandOnce();
  
  vector<unsigned int> best_i_inliers;
  int current_it = 0;
  
  while(current_it < max_its)
  {
    PnP.reset_correspondences();
    
    int i_corr[N];
    int n = 0;
    while(n < N)
    {
      i_corr[n] = Random::RandomInt(0, model_indices.size()-1);
      
      if( (n == 0) || (find(i_corr, i_corr + n, i_corr[n]) == i_corr + n) )
      {
        int face_idx = model_indices[i_corr[n]];
        int scene_idx = scene_indices[i_corr[n]];
        
        const PLYPoint &p3d = face.plypoints[face_idx];
        const cv::KeyPoint kp = scene_surfset.keys[scene_idx];
        
        PnP.add_correspondence(p3d.x, p3d.y, p3d.z, kp.pt.x, kp.pt.y);
        
        ++n;
      }
    }
        
    PnP.compute_pose(cRo_r, cto_r);

    cv::projectPoints(oP_Nx3, cRo, cto, A, K, vec_object_2d_points);
    
    cv::Mat object_2d_points_Nx2(oP_Nx3.rows, 2, CV_32F, 
      &vec_object_2d_points[0]);
        
    // calculate distance from scene point to reprojected point
    projected_distance_xy_Nx2 = object_2d_points_Nx2 - scene_2d_points_Nx2;
    cv::pow(projected_distance_xy_Nx2, 2, projected_distance_xy_Nx2); 
    
    cv::reduce(projected_distance_xy_Nx2, projected_sq_distance_Nx1, 1, 
      CV_REDUCE_SUM);
    // projected_sq_distance is now a col with the square distances of
    // each projected point to its matched scene point
    
    // get the inliers
    cv::compare(projected_sq_distance_Nx1, 
      maxReprojectionError * maxReprojectionError, inlier_status_Nx1, 
      cv::CMP_LE);
    // inlier_status is 0 at outlier positions and 255 at inlier positions
    
    const unsigned char *p_inliers = inlier_status_Nx1.ptr<unsigned char>();
    i_inliers.resize(0);
    for(unsigned int i = 0; i < model_indices.size(); ++i)
    {
      if(p_inliers[i] != 0)
      {
        i_inliers.push_back(i);
      }
    }

    // check if we have enough inliers
    if(i_inliers.size() > best_i_inliers.size() && 
      (int)i_inliers.size() >= MIN_POINTS)
    {
      best_i_inliers = i_inliers;
      
      // update iterations
      double w = (double)best_i_inliers.size() / (double)model_indices.size();
      const float p = 0.95;
      int its = (int)( log(1 - p) / log(1 - pow(w, N)) );
      if(its < max_its) max_its = its;
    }

    current_it++;
  }
  
  if((int)best_i_inliers.size() >= MIN_POINTS)
  {
    // we have found the object
    
    // get the indices of the inliers
    vector<int> inlier_model_indices, inlier_scene_indices;
    vector<float> inlier_distances;
    {
      const unsigned int M = best_i_inliers.size();

      inlier_model_indices.resize(M);
      inlier_scene_indices.resize(M);
      inlier_distances.resize(M);
      
      for(unsigned int i = 0; i < M; ++i)
      {
        inlier_model_indices[i] = model_indices[ best_i_inliers[i] ];
        inlier_scene_indices[i] = scene_indices[ best_i_inliers[i] ];
        inlier_distances[i] = distances[ best_i_inliers[i] ];
      }
    }
    
    cv::Mat _image_to_show;
    IF_DEBUG_OR_VISUALIZATION_MODE
    (
      DUtilsCV::Drawing::drawCorrespondences(_image_to_show, data.img,
        face.image, scene_surfset.keys, face.surf.keys,
        inlier_scene_indices, inlier_model_indices);  
      
      IF_DEBUG_MODE
      ( 
        std::string filename;
        if(doReChecking)
          filename = m_debug_prefix + "1_0_first_pnp.png";
        else
          filename = m_debug_prefix + "1_1_re_pnp.png";
          
        cv::imwrite(filename, _image_to_show); 
      )
    )
    
    if(doReChecking)
    {
      bool found = detectWithPnP(scene_surfset, model, face, 
        inlier_model_indices, inlier_scene_indices, inlier_distances, 
        data, 1, false, detection);
      
      if(found) return true;
    }
    
    if(visualizationMode())
    {
      visualize(_image_to_show);
    }
    
    // get the final transformation now
    cv::Mat cTo;
    
    calculatePose(face, inlier_model_indices, scene_surfset, 
      inlier_scene_indices, data.params.camera, cTo);
        
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
    } // if cTo is ok
    
  } // if(best_i_inliers.size() >= MIN_POINTS)
  
  return false;
}

// ---------------------------------------------------------------------------


