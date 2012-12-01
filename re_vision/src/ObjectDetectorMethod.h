/** \file ObjectDetectorMethod.h
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

#ifndef __OBJECT_DETECTOR_METHOD__
#define __OBJECT_DETECTOR_METHOD__

#include <opencv/cv.h>
#include <vector>
#include "re_msgs/DetectedObject.h"
#include "ObjectModel.h"
#include "CameraBridge.h"
#include "VisualizationManager.h"

#include "DVision.h"


class ObjectDetectorMethod
{  
public:

  /// Stores original image and processed data that can be shared between
  /// several object detector methods.
  /// Apart from the image, other data can be provided. All of these must come
  /// with a flag to say whether the data have been already computed
  class DetectionData
  {
  public:
  
    // All the data fields must inherit AnyData
    struct AnyData
    {
      bool provided;
      AnyData(): provided(false){}
    };
  
    struct SURFdata: public AnyData
    {
      DVision::SurfSet surfset;
    };
    
    // General parameters
    struct Parameters
    {
      const CameraBridge &camera; // camera intrinsic parameters
      
      Parameters(const CameraBridge& _cam): camera(_cam){}
    };
    
  public:
    const cv::Mat &img;    // reference image to search for objects
    Parameters params;     // general parameters
    SURFdata surfdata;     // surfs from img
  
  public:
    DetectionData(const cv::Mat &_img, const CameraBridge &_cam)
    : img(_img), params(_cam) {}
    
  };
  
public:

  /**
   * Tries to detect in the given data the given model. Results are stored
   * in detection
   * @param data
   * @param model model to search for
   * @param detection results
   */
  virtual void detect(ObjectDetectorMethod::DetectionData &data, 
    ObjectModel &model,
    re_msgs::DetectedObject &detection) = 0;
  
  /**
   * Sets a flag to say if debug information should be generated
   * @param onoff debug mode iif true
   */
  virtual inline void setDebugMode(bool onoff, const std::string &dir)
  {
    m_debug = onoff;
    m_debug_dir = dir;
  }
  
  /**
   * Says whether the debug mode is on
   * @return true iif debug mode is on
   */
  virtual inline bool debugMode() const { return m_debug; }  

  /**
   * Sets a visualization manager for this method
   * @param vis
   */
  virtual inline void setVisualizationManager(VisualizationManager *vis)
  {
    m_visualizer = vis;
  }
  
  /** 
   * Returns whether visualization must be done
   */
  virtual inline bool visualizationMode() const { return m_visualizer != NULL; }

protected:
  /// Visualization functions

  /** 
   * Creates the detection method with a visualization manager
   * @param vis 
   */
  ObjectDetectorMethod(VisualizationManager *vis = NULL):
    m_visualizer(vis){}
  
  /** 
   * Visualizes an image
   * @param image
   */
  inline void visualize(const cv::Mat &image, const std::string &text = "", 
    bool hold = false)
  { 
    if(m_visualizer) m_visualizer->show(image, text, hold);
  }

protected:
  /// Functions for several algorithms
  
  /**
   * Calculates the 3D pose of the model face in the image.
   * Assumes that the keypoints come from a rectified image
   * @param face model face
   * @param face_indices indices of correspondences in face
   * @param scene_surfset features in the scene image
   * @param scene_indices indices of correspondences in the scene
   * @param camera camera intrinsic parameters
   * @param cTo (out) returned 4x4 matrix transformation from camera to object
   */
  void calculatePose(const ObjectModel::Face &face, const std::vector<int>
    &face_indices, const DVision::SurfSet &scene_surfset, 
    const std::vector<int> &scene_indices, const CameraBridge &camera,
    cv::Mat &cTo) const;

  /**
   * Converts a 4x4 transformation matrix into a ROS pose data structure
   * @param T transformation matrix
   * @param pose ROS pose
   */
  void convertPose(const cv::Mat &T, geometry_msgs::Pose &pose) const;
  
  /**
   * Converts 3D points of a face into the camera reference
   * @param face
   * @param indices indices of face points involved in
   * @param cTo transformation from camera to object
   * @param points3d returning points3d in the camera reference
   */
  void convert3DPoints(const ObjectModel::Face &face, 
    const std::vector<int> &indices, 
    const cv::Mat &cTo, 
    std::vector<geometry_msgs::Point> &points3d) const;
  
  /**
   * Converts all the 3D points of a face into the camera reference
   * @param face
   * @param cTo transformation from camera to object
   * @param points3d returning pointsd in the camera reference
   */
  void convert3DPoints(const ObjectModel::Face &face,
    const cv::Mat &cTo,
    std::vector<geometry_msgs::Point> &points3d) const;
  
  /** 
   * Copies all the model 3d points from the given face
   * @param face
   * @param points3d: points in the object reference
   */
  void get3DModelPoints(const ObjectModel::Face &face, 
    std::vector<geometry_msgs::Point> &points3d) const;
  
  /**
   * Projects all the 3D points of the face into the image given the
   * pose of the object and the parameters of the camera
   * Note: this function does not check if the 2d points are within the
   * bounds of the image
   * Note: that is done in ObjectDetectorProvider::rectifyDetections
   * @param face detected face of the object
   * @param cTo transformation from camera to object
   * @param camera intrinsic camera parameters
   * @param points2d all the 3d points of the face projected into the camera
   */
  void project2DPoints(const ObjectModel::Face &face, 
    const cv::Mat &cTo, const CameraBridge &camera,
    std::vector<re_msgs::Pixel> &points2d) const;
  
  /**
   * Changes the orientation of a transformation from the vision reference
   * system to the robotics reference system.
   * In vision: x right, y down, z forward
   * In robotics: x forward, y left, z up
   * @param cTo a transformation in vision orientaiton
   * @param rTo same transformation in robotics orientation
   */
  void changeOrientation(const cv::Mat &cTo, cv::Mat &rTo) const;

protected:

  bool m_debug; /// debug mode
  std::string m_debug_dir; /// debug dir
  
  VisualizationManager *m_visualizer;
  
};

#endif
