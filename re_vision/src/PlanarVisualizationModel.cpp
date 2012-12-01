/** \file PlanarVisualizationModel.cpp
 * \brief Visualization model of a planar object
 *
 * Class to store a planar object visualization model
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
#include "PlanarVisualizationModel.h"
#include "ObjectModel.h"
#include <cv.h>
#include <highgui.h>
#include <string>

#include "DUtilsCV.h"

using namespace std;

// ----------------------------------------------------------------------------

PlanarVisualizationModel::PlanarVisualizationModel
  (const std::vector<ObjectModel::Face> &faces, 
   const MetaFile::MetaData::tDimensions::tPlanar &dim)
{
  m_face_images.resize(faces.size());
  m_face_images_bw.resize(faces.size());
  m_inv_A.resize(faces.size());
  
  m_dim = dim;

  for(unsigned int i = 0; i < faces.size(); ++i)
  {
    m_face_images[i] = faces[i].image.clone();
    if(m_face_images[i].channels() == 1)
      m_face_images_bw[i] = m_face_images[i];
    else
      cv::cvtColor(m_face_images[i], m_face_images_bw[i], CV_RGB2GRAY);
    
    if(faces[i].A.type() == CV_64F)
      m_inv_A[i] = faces[i].A.inv();
    else
    {
      cv::Mat AinvF = faces[i].A.inv();
      AinvF.convertTo(m_inv_A[i], CV_64F);
    }
  }

}

// ----------------------------------------------------------------------------

void PlanarVisualizationModel::draw(cv::Mat &img, const cv::Mat &cTo,
  const cv::Mat &A, const cv::Mat &K) const
{
  assert(cTo.type() == CV_64F);
  assert(img.type() == CV_8U || img.type() == CV_8UC3);
  
  // alpha channel of the object
  const float alpha = 0.35f;
  
  cv::Mat k;
  if(K.empty()) 
    k = cv::Mat::zeros(4,1,CV_32F);
  else
    k = K;
  
  // calculate face points in camera reference first
  vector<pair<float, int> > zvalues; // < min z, face idx >  
  zvalues.reserve(m_face_images.size());
  
  for(unsigned int i = 0; i < m_face_images.size(); ++i)
  {
    cv::Mat cTf = cTo * m_dim.Faces[i].oTf;
    // z value of the origin of the patch in the camera frame
    zvalues.push_back(make_pair
      (cTf.at<double>(2,3) / cTf.at<double>(3,3), i ));
  }
  
  // sort in ascending z
  sort(zvalues.begin(), zvalues.end());
  
  // print in descending z
  cv::Mat cRo;
  cv::Mat cto(3, 1, CV_64F);
  DUtilsCV::Transformations::decomposeRt(cTo, cRo, cto);
  
  vector<pair<float, int> >::reverse_iterator zit;
  for(zit = zvalues.rbegin(); zit != zvalues.rend(); ++zit)
  {
    int face_idx = zit->second;
    
    const cv::Mat& model_im = (img.channels() == 3 ? 
      m_face_images[face_idx] : m_face_images_bw[face_idx]);
    
    const float w =  m_dim.Faces[face_idx].Width/2.f;
    const float h = m_dim.Faces[face_idx].Height/2.f;
    cv::Mat fP = (cv::Mat_<double>(4, 4) <<
      -w,  w, w, -w,
      -h, -h, h,  h,
       0,  0, 0,  0,
       1,  1, 1,  1);
    
    cv::Mat oP = m_dim.Faces[face_idx].oTf * fP; // 4x4
    
    cv::Mat oP_4x3(oP.cols, 3, CV_32F);

    for(int c = 0; c < oP.cols; ++c)
      for(int r = 0; r < 3; ++r)
        oP_4x3.at<float>(c, r) = oP.at<double>(r, c) / oP.at<double>(3, c);
    
    vector<cv::Point2f> cam_pixels;
    cv::projectPoints(oP_4x3, cRo, cto, A, k, cam_pixels);
    
    cv::Mat camera_pixels(4, 2, CV_32F, &(cam_pixels[0].x));
    cv::Mat object_pixels = (cv::Mat_<float>(4, 2) <<
                  0,             0,
      model_im.cols,             0,
      model_im.cols, model_im.rows,
                  0, model_im.rows);
        
    cv::Mat cHo = cv::findHomography(object_pixels, camera_pixels, 0);

    if(!cHo.empty())
    {
      if(alpha < 1.f)
      {
        cv::Mat img_original = img.clone();

        // create warp image and copy on img
        cv::warpPerspective(model_im, img, cHo, img.size(), 
          cv::INTER_AREA, cv::BORDER_TRANSPARENT);

        // this could be done better
        
        if(img.channels() == 3)
        {
          for(int y = 0; y < img.rows; ++y)
          {
            unsigned char *py = img.ptr<unsigned char>(y);
            const unsigned char *qy = img_original.ptr<unsigned char>(y);
            
            for(int x = 0; x < img.cols; ++x)
            {
              int x3 = x*3;
              py[x3] = (float)py[x3] * alpha + (1.f - alpha) * (float)qy[x3];
              py[x3+1] = (float)py[x3+1]*alpha + (1.f - alpha)*(float)qy[x3+1];
              py[x3+2] = (float)py[x3+2]*alpha + (1.f - alpha)*(float)qy[x3+2];
            }
          }
        }
        else
        {
          for(int y = 0; y < img.rows; ++y)
          {
            unsigned char *py = img.ptr<unsigned char>(y);
            const unsigned char *qy = img_original.ptr<unsigned char>(y);
            
            for(int x = 0; x < img.cols; ++x)
            {
              py[x] = (float)py[x]*alpha + (1.f - alpha)*(float)qy[x];
            }
          }
        }
        
      }
      else
      {
        // just warp the model on the image
        // create warp image and copy on img
        cv::warpPerspective(model_im, img, cHo, img.size(), 
          cv::INTER_AREA, cv::BORDER_TRANSPARENT);
      }

      // draw a rectangle
      DUtilsCV::Drawing::drawBox(img, cHo, model_im.cols, model_im.rows);
	  }
  } // for each face in descencing z
  
  // draw a reference system
  float length;
  float d[3] = {m_dim.Width, m_dim.Height, m_dim.Depth};
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

  DUtilsCV::Drawing::drawReferenceSystem(img, cRo, cto, A, k, length);
  
}

// ----------------------------------------------------------------------------

void PlanarVisualizationModel::draw(cv::Mat &img, const cv::Mat &sHm,
  unsigned int face_idx) const
{
  if(face_idx >=  m_face_images.size()) return;
  
  const cv::Mat& model_im = (img.channels() == 3 ? 
    m_face_images[face_idx] : m_face_images_bw[face_idx]);
  
  // create warp image and copy on img
  cv::warpPerspective(model_im, img, sHm, img.size(), 
    cv::INTER_AREA, cv::BORDER_TRANSPARENT);

  // draw a rectangle
  DUtilsCV::Drawing::drawBox(img, sHm, model_im.cols, model_im.rows);
  
}

// ----------------------------------------------------------------------------


