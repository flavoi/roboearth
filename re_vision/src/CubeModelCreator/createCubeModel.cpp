/** \file createCubeModel.cpp
 * \brief Creates a rectangular cuboid model
 *
 * Creates a recognition model of an object composed of several planes that
 * form a rectangular cuboid. The model is defined by its faces and its 
 * width, height and depth.
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
 * Usage: createCubeModel <object name> <Wo> <Ho> <Do> <output model dir>
 *      {<patch A> <wa> <ha> | - } [ ... [ {<patch F> <wf> <hf> | - } ] ]
 *
 * object name
 * Wo, Ho, Do: real dimensions of the rectangular cuboid in metres (width,
 *   height and depth)
 * output model dir: directory to store the model files
 * patch X: path to the undistorted image of face X.
 * wX, hX: real dimensions in metres of the image given for the face
 * 
 * The order of the faces and their orientation is documented here:
 * http://www.roboearth.org/wiki/Re_vision:_Creating_object_recognition_models
 *
 * Note: A patch can be skipped with a hyphen -
 * Note: If the real dimensions of a patch are bigger than those of the 
 * corresponding face of the object, the image is cropped (anchor in the center) 
 * to make its dimensions consistent with the object.
 * If the real dimensions of the patch are smaller than those of the object,
 * the patch is assumed to be in the center of the corresponding face of the
 * object.
 *
 */

#include "ros/ros.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../MetaFile.h"

#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"

typedef DVision::PMVS::PLYFile PLYFile;
typedef DVision::PMVS::PLYFile::PLYPoint PLYPoint;
typedef DVision::Bundle::CameraFile::Camera Camera;

using namespace std;

// ---------------------------------------------------------------------------

struct tParamsPatch
{
  bool is_present;
  std::string image_filename;
  float w, h;
};

struct tParams
{
  std::string object_name;
  std::string out_model_dir;
  float W, H, D;
  tParamsPatch patches[6];
  int hessian;
};

// ---------------------------------------------------------------------------

/**
 * Generates the 3D points of the given surf features. The 3D points are given
 * in the object reference, located in the center of the object, x pointing right,
 * y pointing down, z going away the camera.
 * @param img source color image
 * @param width
 * @param height dimensions of the real object in metres. One dimension can be
 *   zero
 * @param oTp transformation from the object frame to this patch frame
 * @param surfset surfs extracted from img
 * @param plypoint created 3d points
 */
void createPLYpoints(const cv::Mat &img, float width, float height,
  const cv::Mat& oTp,
  const DVision::SurfSet &surfset, std::vector<PLYPoint> &plypoints);

/**
 * Generates under the directory model_dir the files of one face of the model
 * @param model_dir output model directory
 * @param face_idx index of this face
 * @param img face image
 * @param surfs surfs from img
 * @param plypoints 3d coords of the surfs keypoints
 * @param camera intrinsic parameters of camera
 */
void saveFace(const std::string &model_dir, int face_idx, const cv::Mat &img,
  const DVision::SurfSet &surfs, const std::vector<PLYPoint> &plypoints,
  const Camera &camera);

/**
 * Parses the parameters given to the program
 * @param argc
 * @param argv
 * @param params parameters obtained
 * @return true iif params are ok
 */
bool parseParams(int argc, char *argv[], tParams &params);

/**
 * Crops the given image so that it does not exceed the maximum dimensions
 * @param im image
 * @param current_width (in/out)
 * @param current_height (in/out) current dimensions in metres of the image.
 *   They are updated with the final width and height
 * @param max_width
 * @param max_height desired dimensions in metres
 */
void cropImage(cv::Mat &im, float& current_width, float& current_height,
  float max_width, float max_height);

/** 
 * Computes the transformations to convert points from the general object
 * reference to each patch local reference
 * @param oTps vector with the 6 transformations in order (from A to F)
 * @param W
 * @param H
 * @param D real dimensions of the object
 */
void calculatePatchTransformations(std::vector<cv::Mat> &oTps,
  float W, float H, float D);

/**
 * Creates a virtual camera that would watch a patch in the given pose
 * The virtual camera is suposed to be oriented as the patch, at a distance
 * of 1 metre
 * @param im image
 * @param width
 * @param height real dimensions of the given image
 * @param oTp pose of patch in the object frame
 * @param camera virtual camera
 */
void createCamera(const cv::Mat &im, float width, float height,
  const cv::Mat &oTp, Camera &camera);

// ---------------------------------------------------------------------------

static const int DEFAULT_HESSIAN_THRESHOLD = 300;

// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "createCubeModel");
 
  tParams params;
  if(!parseParams(argc, argv, params))
  {
    ROS_WARN("Usage: createCubeModel <object name> <Wo> <Ho> <Do> "
      "<output model dir> "
      "{<patch A> <wa> <ha> | - } [ ... [ {<patch F> <wf> <hf> | - } "
      "[<surf threshold>] ] ]");
    return 1;
  }

  // max real width of each patch according to its position in the cuboid
  // and the real dimensions of the object
  float max_patch_width[] = { 
    params.W, // patch A
    params.D, // patch B
    params.W, // patch C
    params.D, // patch D
    params.W, // patch E
    params.W  // patch F
  };
  
  // idem for height
  float max_patch_height[] = { 
    params.H, // patch A
    params.H, // patch B
    params.H, // patch C
    params.H, // patch D
    params.D, // patch E
    params.D  // patch F
  };

  ROS_INFO("Computing the object coordinate frame...");
  vector<cv::Mat> oTps;
  calculatePatchTransformations(oTps, params.W, params.H, params.D);

  if(!DUtils::FileFunctions::DirExists(params.out_model_dir.c_str()))
    DUtils::FileFunctions::MkDir(params.out_model_dir.c_str());

  MetaFile::MetaData meta;

  int face_idx = 0;
  for(int patch_idx = 0; patch_idx < 6; ++patch_idx)
  {
    if(params.patches[patch_idx].is_present)
    {
      ROS_INFO(":: Patch %c", 'A' + patch_idx);
      const tParamsPatch& pp = params.patches[patch_idx];
      
      ROS_INFO("Reading image...");
      cv::Mat im = cv::imread(pp.image_filename, 1);
      
      float max_width = max_patch_width[patch_idx];
      float max_height = max_patch_height[patch_idx];
      
      float real_width = pp.w;
      float real_height = pp.h;
      
      if(real_width > max_width || real_height > max_height)
      {
        ROS_INFO("Patch size is %.3fx%.3f metres, but the maximum size is "
          "%.3fx%.3f. Cropping...", 
          real_width, real_height, max_width, max_height);
        
        cropImage(im, real_width, real_height, max_width, max_height);
      }
      
      ROS_INFO("Extracting SURFs (hessian threshold: %d)...", params.hessian);
      cv::Mat bw_img;
      if(im.depth() == 1) bw_img = im;
      else cv::cvtColor(im, bw_img, CV_BGR2GRAY);
      
      DVision::SurfSet surfset;
      surfset.Extract(bw_img, params.hessian);
      ROS_INFO("  %d points obtained", surfset.size());

      ROS_INFO("Computing 3D coords and camera info...");
      vector<PLYPoint> plypoints;
      createPLYpoints(im, real_width, real_height, oTps[patch_idx],
        surfset, plypoints);
      
      Camera camera;
      createCamera(im, real_width, real_height, oTps[patch_idx], camera);
      
      ROS_INFO("Saving face %d from patch %c...", face_idx, 'A' + patch_idx);
      saveFace(params.out_model_dir, face_idx, im, surfset, plypoints, camera);
      
      // save dimension info
      meta.Dimensions.Planar.Faces.push_back(MetaFile::MetaData::tFaceDim());
      MetaFile::MetaData::tFaceDim &face_dim = 
        meta.Dimensions.Planar.Faces.back();
      face_dim.Width = real_width;
      face_dim.Height = real_height;
      face_dim.oTf = oTps[patch_idx];
      
      face_idx++;
    }
  }
  
  ROS_INFO("Saving meta data...");
  meta.Name = params.object_name;
  meta.Type = "planar";
  meta.NFaces = meta.Dimensions.Planar.Faces.size();
  meta.Dimensions.Planar.Width = params.W;
  meta.Dimensions.Planar.Height = params.H;
  meta.Dimensions.Planar.Depth = params.D;
  
  MetaFile::saveFile(params.out_model_dir + "/meta.xml", meta);
  
  ROS_INFO("Done");
  
  return 0;
}

// ---------------------------------------------------------------------------

bool parseParams(int argc, char *argv[], tParams &params)
{
  if(argc < 9) return false;
  
  params.object_name = argv[1];
  params.W = atof(argv[2]);
  params.H = atof(argv[3]);
  params.D = atof(argv[4]);
  params.out_model_dir = argv[5];
  params.hessian = DEFAULT_HESSIAN_THRESHOLD;
  
  bool some_present = false;
  int patch_idx = 0;
  int i = 6;
  while(i < argc)
  {
    if(patch_idx == 6)
    {
      // reading surf threshold
      params.hessian = atoi(argv[i]);
      ++i;
    }
    else
    {
      // reading a face
      std::string s = argv[i];
      if(s == "-")
      {
        params.patches[patch_idx].is_present = false;
        ++patch_idx;
        ++i;
      }
      else
      {
        if(i + 2 < argc)
        {
          params.patches[patch_idx].is_present = true;
          some_present = true;
          
          params.patches[patch_idx].image_filename = argv[i];
          params.patches[patch_idx].w = atof(argv[i+1]);
          params.patches[patch_idx].h = atof(argv[i+2]);

          i += 3;
          ++patch_idx;
        }
        else{ return false; }
      }
    }
  }
  
  for(; patch_idx < 6; ++patch_idx) 
    params.patches[patch_idx].is_present = false;
  
  return some_present;
}

// ---------------------------------------------------------------------------

void calculatePatchTransformations(std::vector<cv::Mat> &oTps, 
  float W, float H, float D)
{
  oTps.clear();
  
  const float w2 = W/2.f;
  const float h2 = H/2.f;
  const float d2 = D/2.f;

  // A
  oTps.push_back( DUtilsCV::Transformations::transl(0,0, -d2) );
  // B
  oTps.push_back( DUtilsCV::Transformations::roty(-M_PI/2, w2, 0, 0) );
  // C 
  oTps.push_back( DUtilsCV::Transformations::rotz(M_PI)
    * DUtilsCV::Transformations::rotx(M_PI, 0, 0, d2) );
  // D
  oTps.push_back( DUtilsCV::Transformations::roty(M_PI/2, -w2, 0, 0) );
  // E 
  oTps.push_back( DUtilsCV::Transformations::rotx(-M_PI/2, 0, -h2, 0) );
  // F
  oTps.push_back( DUtilsCV::Transformations::rotx(M_PI/2, 0, h2, 0) );
}

// ---------------------------------------------------------------------------

void cropImage(cv::Mat &im, float& current_width, float& current_height,
  float max_width, float max_height)
{
  float pxperm = 
    ( ((float)im.cols / current_width) + ((float)im.rows / current_height) )/2;
  
  int cols = im.cols;
  int rows = im.rows;
  bool crop = false;
  
  if(current_width > max_width)
  {
    cols = max_width * pxperm;
    current_width = max_width;
    crop = true;
  }
  
  if(current_height > max_height)
  {
    rows = max_height * pxperm;
    current_height = max_height;
    crop = true;
  }
  
  if(crop)
  {
    cv::Mat aux;
    cv::getRectSubPix(im, cv::Size(cols, rows), 
      cv::Point2f((float)im.cols/2.f, (float)im.rows/2.f), aux);
    im = aux;
  }
  
}

// ---------------------------------------------------------------------------

void createCamera(const cv::Mat &im, float width, float height,
  const cv::Mat &oTp, Camera &camera)
{
  cv::Mat cTp = DUtilsCV::Transformations::transl(0, 0, 1);
  cv::Mat cTo = cTp * DUtilsCV::Transformations::inv(oTp);
  
  float fx = im.cols / width;
  float fy = im.rows / height;
  camera.f = (fx + fy) / 2;
  camera.k1 = camera.k2 = 0;
  
  camera.t = cv::Mat::zeros(3, 1, CV_64F);
  DUtilsCV::Transformations::decomposeRt(cTo, camera.R, camera.t);
}

// ---------------------------------------------------------------------------

void createPLYpoints(const cv::Mat &img, float width, float height,
  const cv::Mat &oTp,
  const DVision::SurfSet &surfset, std::vector<PLYPoint> &plypoints)
{
  plypoints.resize(0);
  if(surfset.keys.empty()) return;

  double mperpixel;
  if(width > 0.f && height > 0.f)
    mperpixel = ((width / (float)img.cols) + (height / (float)img.rows)) / 2.f;
  else if(width > 0.f) mperpixel = width / (float)img.cols;
  else mperpixel = height / (float)img.rows;
  
  const float cx = (float)img.cols / 2.f;
  const float cy = (float)img.rows / 2.f;
  
  cv::Mat pP(4, surfset.size(), CV_64F);
  
  vector<cv::KeyPoint>::const_iterator kit;
  for(kit = surfset.keys.begin(); kit != surfset.keys.end(); ++kit)
  {
    int i = kit - surfset.keys.begin();
    pP.at<double>(0, i) = (kit->pt.x - cx) * mperpixel;
    pP.at<double>(1, i) = (kit->pt.y - cy) * mperpixel;
    pP.at<double>(2, i) = 0.;
    pP.at<double>(3, i) = 1.;
  }
  
  cv::Mat oP = oTp * pP;
  cv::Mat oN = oTp.col(2); // normal of patch: oTp * [0 0 1 0]'
  
  vector<cv::Mat> bgr_planes;
  cv::split(img, bgr_planes);
  
  plypoints.reserve(surfset.size());
  
  for(kit = surfset.keys.begin(); kit != surfset.keys.end(); ++kit)
  {
    int i = kit - surfset.keys.begin();
    
    plypoints.push_back(PLYPoint());
    PLYPoint &p = plypoints.back();
 
    p.x = oP.at<double>(0, i) / oP.at<double>(3, i);
    p.y = oP.at<double>(1, i) / oP.at<double>(3, i);
    p.z = oP.at<double>(2, i) / oP.at<double>(3, i);
    
    p.nx = oN.at<double>(0, 0);
    p.ny = oN.at<double>(1, 0);
    p.nz = oN.at<double>(2, 0);

    p.r = bgr_planes[2].at<uchar>(kit->pt.y, kit->pt.x);
    p.g = bgr_planes[1].at<uchar>(kit->pt.y, kit->pt.x);
    p.b = bgr_planes[0].at<uchar>(kit->pt.y, kit->pt.x);
  }
  
}

// ---------------------------------------------------------------------------

void saveFace(const std::string &model_dir, int face_idx, const cv::Mat &img,
  const DVision::SurfSet &surfs, const std::vector<PLYPoint> &plypoints,
  const Camera &camera)
{
  stringstream prefix;
  prefix << model_dir << "/face_" << setw(3) << setfill('0') << face_idx;
  
  cv::imwrite(prefix.str() + ".png", img);
  surfs.Save(prefix.str() + ".key.gz");
  PLYFile::saveFile(prefix.str() + ".ply", plypoints);
  camera.save(prefix.str() + ".txt", 
    "Fake camera. Assumes cx=w/2, cy=h/2, Z_object=1");
}

// ---------------------------------------------------------------------------

