/** \file ObjectModel.cpp
 * \brief Recognition model of an object
 *
 * Class to store a general object recognition model
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

#include "ObjectModel.h"
#include "MetaFile.h"
#include "PlanarVisualizationModel.h"
#include "PointCloudVisualizationModel.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

#include <opencv/cv.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <algorithm>

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;
typedef DVision::Bundle::CameraFile CameraFile;
typedef DVision::Bundle::CameraFile::Camera Camera;
typedef DVision::SurfSet SurfSet;

using namespace std;

// ----------------------------------------------------------------------------

ObjectModel::ObjectModel():
  m_visualization_model(NULL)
  //, m_flann(NULL), m_features(NULL), 
{

}

// ----------------------------------------------------------------------------

ObjectModel::ObjectModel(const std::string &dir,
  bool load_visualization_model):
  m_visualization_model(NULL)
  //, m_flann(NULL), m_features(NULL), 
{
  loadDirectory(dir, load_visualization_model);
}

// ----------------------------------------------------------------------------

ObjectModel::~ObjectModel()
{
  //delete m_features;
  //delete m_flann;
  
  delete m_visualization_model;
  m_visualization_model = NULL;

#if !OPENCV_ADV_MATCHER
  vector<Face>::iterator fit;
  for(fit = Faces.begin(); fit != Faces.end(); ++fit)
  {
    //delete fit->flann_matcher;
    //fit->flann_matcher = NULL;
  }
#endif

}

// ----------------------------------------------------------------------------

void ObjectModel::loadDirectory(const std::string &dir, 
  bool load_visualization_model)
{
  const int N = 1024;
  char buffer[N];

  this->Name = this->Type = "";
  this->Faces.clear();
  //this->m_map.clear();
  //delete m_flann; // ####
  
  snprintf(buffer, N, "%s/meta.xml", dir.c_str());
  
  MetaFile::MetaData data;
  MetaFile::readFile(buffer, data);

  this->Name = data.Name;
  this->Type = data.Type;
  
  /*
  snprintf(buffer, N, "%s/index_tree.", dir.c_str());
  //if(DUtils::FileFunctions::FileExists(buffer))
  // ####
  if(true)
  {
    loadFlann(string(buffer) + "fln", string(buffer) + "key.gz"); // ###
  
    m_map.resize(0);
    m_map.resize(m_flann->size(), std::vector<vector<int> >(data.NFaces)); // ###
  }
  */
  
  this->Faces.reserve(data.NFaces); // avoid reallocations (important!)
  for(int i = 0; i < data.NFaces; ++i)
  {
    this->Faces.push_back(Face());
    Face& face = this->Faces.back();
    
    snprintf(buffer, N, "%s/face_%03d.png", dir.c_str(), i);
    face.image = cv::imread(buffer);
    if(face.image.empty()) 
      throw string("ObjectModel: cannot load image ") + buffer;
    
    snprintf(buffer, N, "%s/face_%03d.key.gz", dir.c_str(), i);
    face.surf.Load(buffer);
        
    snprintf(buffer, N, "%s/face_%03d.ply", dir.c_str(), i);
    PLYFile::readFile(buffer, face.plypoints);
    
    /*
    snprintf(buffer, N, "%s/face_%03d.idx", dir.c_str(), i);
    //if(DUtils::FileFunctions::FileExists(buffer)) // ###
    if(true)
    {
      updateMap(buffer, i, face.surf.keys.size()); // ###
    }
    */
    
    snprintf(buffer, N, "%s/face_%03d.txt", dir.c_str(), i);
    Camera camera(buffer);

    face.A = (cv::Mat_<float>(3, 3) <<
      camera.f, 0, face.image.cols/2.f,
      0, camera.f, face.image.rows/2.f,
      0, 0, 1);
    
    face.cRo = camera.R;
    face.cto = camera.t;
    
    // create flann matcher (it cannot be stored!)
    vector<cv::Mat> flann_training_desc;
    
    if(!face.surf.keys.empty())
    {
      flann_training_desc.push_back(cv::Mat
        (face.surf.keys.size(), face.surf.GetDescriptorLength(), CV_32F,
        &(*face.surf.descriptors.begin())  )); // shared data
      // note: these data will be used by the flann matcher (it stores a pointer)
      // it is important not to reallocate them
    }

#if OPENCV_ADV_MATCHER
    face.flann_matcher.add(flann_training_desc);
    face.flann_matcher.train();
#else
    if(!flann_training_desc.empty()){
      face.flann_matcher = new cv::flann::Index(flann_training_desc[0], 
        cv::flann::KDTreeIndexParams(4));
    }else{
      face.flann_matcher = NULL;
    }
#endif
  }

  delete m_visualization_model;
  m_visualization_model = NULL;
  
  if(load_visualization_model)
  {
    if(Type == "planar")
    {
      m_visualization_model = new PlanarVisualizationModel
        (this->Faces, data.Dimensions.Planar);
    }
    else if(Type == "3D")
    {
      snprintf(buffer, N, "%s/drawing_model.ply", dir.c_str());
      m_visualization_model = new PointCloudVisualizationModel(buffer);
    }
  }
}

// ----------------------------------------------------------------------------

bool ObjectModel::checkDirectory(const std::string &dir)
{
  const int N = 1024;
  char buffer[N];
  
  MetaFile::MetaData data;
  
  try {
    snprintf(buffer, N, "%s/meta.xml", dir.c_str());
    MetaFile::readFile(buffer, data);
  }catch(std::string ex)
  {
    return false;
  }
    
  for(int i = 0; i < data.NFaces; ++i)
  {
    snprintf(buffer, N, "%s/face_%03d.png", dir.c_str(), i);
    if(!DUtils::FileFunctions::FileExists(buffer)) return false;
    
    snprintf(buffer, N, "%s/face_%03d.key.gz", dir.c_str(), i);
    if(!DUtils::FileFunctions::FileExists(buffer)) return false;
        
    snprintf(buffer, N, "%s/face_%03d.ply", dir.c_str(), i);
    if(!DUtils::FileFunctions::FileExists(buffer)) return false;
        
    snprintf(buffer, N, "%s/face_%03d.txt", dir.c_str(), i);
    if(!DUtils::FileFunctions::FileExists(buffer)) return false;
  }
    
  return true;
}

// ----------------------------------------------------------------------------

std::string ObjectModel::getName(const std::string &dir)
{
  stringstream ss;
  ss << dir << "/meta.xml";
  
  MetaFile::MetaData data;
  try {
    MetaFile::readFile(ss.str().c_str(), data);
    return data.Name;
  }catch(std::string ex)
  {
    return "";
  }
}

// ----------------------------------------------------------------------------

/*
void ObjectModel::loadFlann(const std::string &flan_filename,
  const std::string &keys_filename)
{
  delete m_features;
  m_features = new SurfSet;
  m_features->Load(keys_filename);
  
  if(m_flann) delete m_flann;  // ###
  
  cv::Mat features(m_features->keys.size(), 
    m_features->GetDescriptorLength(), CV_32F,
    &m_features->descriptors[0]);
    
  m_flann = new cv::flann::Index(features, 
    cv::flann::SavedIndexParams(flan_filename));
}
*/

// ----------------------------------------------------------------------------

/*
void ObjectModel::updateMap(const char *filename, int face_idx, int N)
{
  fstream f(filename, ios::in);
  
  int global_key_idx;
  for(int i = 0; i < N; ++i)
  {
    // m_map[ global_key_idx ][ face_idx ] = [ local_idx, ... ]
    f >> global_key_idx;
    m_map[global_key_idx][face_idx].push_back(i);
  }
  
  f.close();
}
*/

// ----------------------------------------------------------------------------

struct faceMatches
{
  int face_idx;
  std::vector<int> *scene_indices;
  std::vector<int> *face_indices;
  std::vector<float> *distances;
  
  faceMatches(): scene_indices(NULL), face_indices(NULL), distances(NULL){}
  
  void allocate()
  {
    scene_indices = new std::vector<int>();
    face_indices = new std::vector<int>();
    distances = new std::vector<float>();
  }
  
  void deallocate()
  {
    delete scene_indices;
    delete face_indices;
    delete distances;
  }
};

static bool descendingNumberOfDetections(
  const faceMatches &a, const faceMatches &b)
{
  return a.face_indices->size() > b.face_indices->size();
}

void ObjectModel::detectFaces(const SurfSet &scene, 
  std::vector<int>& face_indices,
  std::vector<std::vector<int> >& key_indices,
  std::vector<std::vector<int> >& scene_indices,
  std::vector<std::vector<float> >& distances,
  int min_detected_points, int max_correspondences_per_point,
  float max_ratio)
{
  // Usual implementation: no repeated matches (max_correspondences_per_point
  // is ignored), matching by neighbor ratio

  face_indices.resize(0);
  key_indices.resize(0);
  scene_indices.resize(0);
  distances.resize(0);
  
  const int N = scene.keys.size(); // number of query descriptors
  const int L = scene.GetDescriptorLength(); // length of descriptors
  
  if(N < 3)
  {
    // no match can be done
    return;
  }
  
  vector<faceMatches> all_face_matches(this->Faces.size());
  // matches[face_idx] = <face_idx, [local_idx, ...]> // all the faces
  for(unsigned int i = 0; i < all_face_matches.size(); ++i)
  {
    all_face_matches[i].face_idx = i;
    all_face_matches[i].allocate();
  }
  
#if OPENCV_ADV_MATCHER
  vector<vector<cv::DMatch> > flann_matches;
#else
  cv::flann::SearchParams flann_params(64);
  cv::Mat flann_indices(N, 2, CV_32S);
  cv::Mat flann_dists(N, 2, CV_32F);
#endif

  // for holding the scene descriptors  
  //cv::Mat queryDescs(N, L, CV_32F);
  cv::Mat queryDescs(N, L, CV_32F, 
    const_cast<float*>(&scene.descriptors[0]));
  
  vector<Face>::iterator fit;
  for(fit = Faces.begin(); fit != Faces.end(); ++fit)
  {
#if !OPENCV_ADV_MATCHER
    if(!fit->flann_matcher) continue; // if no flann-matcher (there were not keys)
#endif

    faceMatches& this_face_match = all_face_matches[fit - Faces.begin()];
    vector<int> &face_indices = *this_face_match.face_indices;
    vector<int> &scene_indices = *this_face_match.scene_indices;
    vector<float> &distances = *this_face_match.distances;

#if OPENCV_ADV_MATCHER
    flann_matches.resize(0);
    fit->flann_matcher.knnMatch(queryDescs, flann_matches, 2);
#else
    fit->flann_matcher->knnSearch(queryDescs, flann_indices, 
      flann_dists, 2, flann_params);
#endif
    
    for(int i = 0; i < N; ++i)
    {
#if OPENCV_ADV_MATCHER
      if(flann_matches[i].size() == 2 && 
        flann_matches[i][0].distance / flann_matches[i][1].distance < max_ratio)
#else
      if(flann_dists.at<float>(i, 0) / flann_dists.at<float>(i, 1) < max_ratio)
#endif
      {
#if OPENCV_ADV_MATCHER
        int idx_in_face = flann_matches[i][0].trainIdx;
        int idx_in_scene = flann_matches[i][0].queryIdx;
        float dist = flann_matches[i][0].distance;
#else
        int idx_in_face = flann_indices.at<int>(i, 0);
        int idx_in_scene = i;
        float dist = flann_dists.at<float>(i, 0);
#endif
        
        // check laplacian sign
        if(scene.laplacians[idx_in_scene] == fit->surf.laplacians[idx_in_face])
        {
        
          // check if there was already a match 
          vector<int>::const_iterator fiit = 
            find(face_indices.begin(), face_indices.end(), idx_in_face);
          
          if(fiit != face_indices.end())
          {
            // check if replacing
            int existing_pos = fiit - face_indices.begin();
            if(dist < distances[existing_pos] )
            {
              // replace
              scene_indices[existing_pos] = idx_in_scene;
              distances[existing_pos] = dist;
            }
          }
          else
          {
            // add match
            face_indices.push_back(idx_in_face);
            scene_indices.push_back(idx_in_scene);
            distances.push_back(dist);
          } // if adding match
        } // if laplacians match
      } // if ratio ok
    } // for each query point    
  } // for each face
  
  // sort face matches in
  sort(all_face_matches.begin(), all_face_matches.end(), 
    descendingNumberOfDetections);
    
  // copy to the final structure those face matches which have enough matches
  face_indices.resize(0); face_indices.reserve(all_face_matches.size());
  key_indices.resize(0); key_indices.reserve(all_face_matches.size());
  scene_indices.resize(0); scene_indices.reserve(all_face_matches.size());
  distances.resize(0); distances.reserve(all_face_matches.size());
  
  for(unsigned int i = 0; i < all_face_matches.size(); ++i)
  {
    const faceMatches &face_matches = all_face_matches[i];
    
    if((int)face_matches.face_indices->size() > min_detected_points)
    {
      face_indices.push_back(face_matches.face_idx);
      
      key_indices.push_back(vector<int>());
      key_indices.back().insert(key_indices.back().end(),
        face_matches.face_indices->begin(), face_matches.face_indices->end());
      
      scene_indices.push_back(vector<int>());
      scene_indices.back().insert(scene_indices.back().end(),
        face_matches.scene_indices->begin(), face_matches.scene_indices->end());

      distances.push_back(vector<float>());
      distances.back().insert(distances.back().end(),
        face_matches.distances->begin(), face_matches.distances->end());
    }
    else{ break; }
  }
  
  // done
  for(unsigned int i = 0; i < all_face_matches.size(); ++i)
    all_face_matches[i].deallocate();

}

// ----------------------------------------------------------------------------

#if 0
// This implementation used the old flann structure
void ObjectModel::detectFaces(const SurfSet &scene, std::vector<int>& face_indices,
      std::vector<std::vector<int> >& key_indices,
      std::vector<std::vector<int> >& scene_indices,
      std::vector<std::vector<float> >& distances,
      int min_detected_points, int max_correspondences_per_point,
      float max_ratio) const
{

#define USE_SEVERAL_MATCHES 1

  face_indices.resize(0);
  key_indices.resize(0);
  
  const int nqueries = scene.keys.size();
  if(nqueries == 0) return;
  
  const int L = scene.GetDescriptorLength();
  
  cv::Mat indices(nqueries, 2, CV_32S);
  cv::Mat dists(nqueries, 2, CV_32F);
  const cv::Mat queries( nqueries, L, CV_32F, 
    const_cast<float*>(&scene.descriptors[0]) );

#if USE_SEVERAL_MATCHES
  int nmatches = max_correspondences_per_point;
  m_flann->knnSearch(queries, indices, dists, nmatches, 
    cv::flann::SearchParams(64));
#else
  m_flann->knnSearch(queries, indices, dists, 2, 
    cv::flann::SearchParams(64));
#endif
  
  vector<faceMatches> matches(this->Faces.size());
  // matches[face_idx] = <face_idx, [local_idx, ...]> // all the faces
  for(unsigned int i = 0; i < matches.size(); ++i)
  {
    matches[i].face_idx = i;
  }
  
  int* indices_ptr = indices.ptr<int>(0);
  float* dists_ptr = dists.ptr<float>(0);
  
#if USE_SEVERAL_MATCHES
  // avoid repeated matches from model to scene
  vector<int> scene_matches;
  vector<int> model_matches;
  vector<float> dist_matches;
  vector<int>::iterator mit;
  
  // scene_matches[i] <--> model_matches[i]
  scene_matches.reserve(nmatches * indices.rows);
  model_matches.reserve(nmatches * indices.rows);
  dist_matches.reserve(nmatches * indices.rows);

  for(int scene_idx = 0; scene_idx < indices.rows; ++scene_idx)
  {
    indices_ptr = indices.ptr<int>(scene_idx);
    dists_ptr = dists.ptr<float>(scene_idx);
    
    for(int j = 0; j < nmatches; ++j)
    {
      int model_idx = indices_ptr[j];
      float dist = dists_ptr[j];
      
      // check if there is alredy a match from model_idx
      //mit = find(model_matches.begin(), model_matches.end(), model_idx);
      if(false)
      //if(mit != model_matches.end())
      {
        // update
        int k = mit - model_matches.begin();
        if(dist < dist_matches[k])
        {
          scene_matches[k] = scene_idx;
          model_matches[k] = model_idx;
          dist_matches[k] = dist;
        }
      }
      else
      {
        // add
        scene_matches.push_back(scene_idx);
        model_matches.push_back(model_idx);
        dist_matches.push_back(dist);
      }
    }
  }
  
  // transform into face information
  
  // matches[face_idx] = <face_idx, [local_idx, ...]> // all the faces
  // m_map[global_key_idx][ face_idx ] = [local_idx, ...]
  
  for(unsigned int i = 0; i < model_matches.size(); ++i)
  {
    int model_idx = model_matches[i];
    int scene_idx = scene_matches[i];
    float dist = dist_matches[i];
    const vector<vector<int> > &map = m_map[model_idx];

    for(unsigned int face_idx = 0; face_idx < this->Faces.size(); ++face_idx)
    {
      matches[face_idx].face_indices.insert(
        matches[face_idx].face_indices.end(),
        map[face_idx].begin(), map[face_idx].end());
      
      matches[face_idx].scene_indices.insert(
        matches[face_idx].scene_indices.end(),
        map[face_idx].size(), scene_idx);
      
      matches[face_idx].distances.insert(
        matches[face_idx].distances.end(),
        map[face_idx].size(), dist);
    }
  }
  
  
#else
  for (int i=0; i < indices.rows; ++i) {
    // indices_ptr[2*i]: model global key index
    int global_key_idx = indices_ptr[2*i];
    if (dists_ptr[2*i] < max_ratio * dists_ptr[2*i+1]) // old
    {
      //git = find(global_key_indices_matched.begin(),
      //  global_key_indices_matched.end(),
      //  global_key_idx);
      //if(git == global_key_indices_matched.end())
      {
        //global_key_indices_matched.push_back(global_key_idx);
        
        const vector<vector<int> > &map = m_map[global_key_idx];
        // m_map[global_key_idx][ face_idx ] = [local_idx, ...]
        
        for(unsigned int face_idx = 0; face_idx < this->Faces.size(); ++face_idx)
        {
          matches[face_idx].face_indices.insert(
            matches[face_idx].face_indices.end(),
            map[face_idx].begin(), map[face_idx].end());
          
          matches[face_idx].scene_indices.insert(
            matches[face_idx].scene_indices.end(),
            map[face_idx].size(), i);
          
          matches[face_idx].distances.insert(
            matches[face_idx].distances.end(),
            map[face_idx].size(), dists_ptr[2*i]);
        }
        
      }
    }
  }
#endif

  // sort in descending order of detected points
  sort(matches.begin(), matches.end(), descendingNumberOfDetections);
  
  // copy those faces with enough points
  face_indices.reserve(matches.size());
  key_indices.reserve(matches.size());
  scene_indices.reserve(matches.size());
  distances.reserve(matches.size());
  
  for(unsigned int i = 0; i < matches.size(); ++i)
  {
    if((int)matches[i].face_indices.size() >= min_detected_points)
    {
      face_indices.push_back(matches[i].face_idx);
      key_indices.push_back(matches[i].face_indices);
      scene_indices.push_back(matches[i].scene_indices);
      distances.push_back(matches[i].distances);
    }
    else{ break; }
  }
  
}
#endif

// ----------------------------------------------------------------------------




