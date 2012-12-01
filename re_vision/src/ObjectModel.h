/** \file ObjectModel.h
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

#ifndef __OBJECT_MODEL__
#define __OBJECT_MODEL__

#include "VisualizationModel.h"
#include "DVision.h"

#include <string>
#include <vector>
#include <opencv/cv.h>

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::SurfSet SurfSet;

// This flag is to distinguish between code for Opencv > 2.1.1, that defined
// cv::FlannBasedMatcher, and code for OpenCV 2.1.1, that only defines a 
// cv::Index class
// Iif OPENCV_ADV_MATCHER is 1, the code for > 2.1.1 will be used
//#define OPENCV_ADV_MATCHER 0 //cturtle
#define OPENCV_ADV_MATCHER 1 //diamondback

class ObjectModel
{
  /** 
   * Important: this class is currently non-copyable because it manages
   * pointers to flann structures that cannot be copied.
   * ToDo: fix this
   * Note: Code for Opencv > 2.1.1 should be copyable (but im not sure)
   */
   
  private:
    // Avoid copies
    ObjectModel (const ObjectModel &);
    ObjectModel & operator = (const ObjectModel &);
    
  public:
    
    ObjectModel();
    
    /** 
     * Creates the model by loading it from a directory
     * @param dir
     * @param load_visualization_model if true, the visualization model
     *   is also loaded
     */
    ObjectModel(const std::string &dir, bool load_visualization_model);
    
    virtual ~ObjectModel();
    
    /**
     * Loads the model from the given directory
     * @param dir
     * @param load_visualization_model if true, the visualization model
     *   is also loaded
     * @throw string exception if error
     * @note dir must contain a meta.xml file and these files for each face:
     *   face_%03d.png, face_%03d.txt, face_%03d.key.gz, face_%03d.ply
     */
    void loadDirectory(const std::string &dir, bool load_visualization_model);
    
    /**
     * Loads only the meta data of a model and returns its name
     * @param dir directory where the model is stored
     * @return object name
     */
    static std::string getName(const std::string &dir);
    
    /** 
     * Checks if a directory contains a valid object model
     * @param dir directory to test
     * @returns true iif dir contains a valid model
     */
    static bool checkDirectory(const std::string &dir);
    
    /**
     * Calculate correspondences between the scene features and all the
     * faces of this model. This only returns those faces, and the indices
     * of their matched keypoints, that have at least min_detected_points
     * correspondences. 
     * @param scene scene features
     * @param face_indices indices of detected faces, in descending order
     *        of number of detected points
     * @param key_indices for each detected face, local indices of its 
     *        matched keypoints, in descending order
     *        of number of detected points
     * @param scene_indices for each detected face, indices of matched
     *        points from the scene, in the same order than key_indices
     * @param min_detected_points faces with fewer matches than this number
     *        are ignored
     * @param max_correspondences_per_point. each scene match can be matched
     *        with up to this number of model points
     * @param max_ratio max ratio between two close neighbors
     */
    void detectFaces(const SurfSet &scene, std::vector<int>& face_indices,
      std::vector<std::vector<int> >& key_indices,
      std::vector<std::vector<int> >& scene_indices,
      std::vector<std::vector<float> >& distances,
      int min_detected_points = 1, int max_correspondences_per_point = 3,
      float max_ratio = 0.6);
    
    /**
     * Returns the visualization model
     */
    inline const VisualizationModel &getVisualizationModel() const
    {
      return *m_visualization_model;
    }
    
  public:
    
    struct Face
    {
      SurfSet surf;
      std::vector<PLYFile::PLYPoint> plypoints;
      cv::Mat image;
      cv::Mat A; // intrinsic parameters of camera
      cv::Mat cRo, cto; // Pose of (o)bject in the (c)amera frame that took the 
                        // face image
      
#if OPENCV_ADV_MATCHER
      // Only valid for OpenCV > 2.1.1
      cv::FlannBasedMatcher flann_matcher;
#else
      cv::flann::Index *flann_matcher;
      
      Face(): flann_matcher(NULL){}
#endif
    };
    
    std::string Name;
    std::vector<Face> Faces;
    std::string Type; // "planar", "3D"
  
  protected:
  
    /** 
     * Reads a .idx file and updates the local maps (m_maps)
     * @param filename
     * @param face_idx current face index
     * @param N number of entries in filename
     */
    //void updateMap(const char *filename, int face_idx, int N);
    
    /**
     * Loads the flann tree
     * @param flan_filename index file
     * @param key_filename feature file
     */
    //void loadFlann(const std::string &flan_filename,
    //  const std::string &keys_filename);
  
  protected:
    /*
    cv::flann::Index *m_flann;
    SurfSet *m_features;
    std::vector<std::vector<std::vector<int> > > m_map; 
    /// m_map[ global_key_idx ][ face_idx ] = [ local_idx_1, local_idx_2, ... ]
    */
    
    VisualizationModel *m_visualization_model;
};

#endif

