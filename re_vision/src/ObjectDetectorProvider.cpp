/** \file ObjectDetectorProvider.cpp
 * \brief Provides the service and topics supported by ObjectDetector
 *
 * Class to provide the SearchFor (object) service and to listen to the
 * new_model topic
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>

#include "re_vision/SearchFor.h"
#include "re_msgs/DetectedObject.h"
#include "ObjectDetectorProvider.h"

#include "ObjectDetectorMethod.h"
#include "ObjectModel.h"
#include "SurfPlanarDetector.h"
#include "Surf3DDetector.h"
#include "VisualizationManager.h"

#include "DUtils.h"


using namespace std;

// ---------------------------------------------------------------------------


ObjectDetectorProvider::ObjectDetectorProvider(
  const ros::NodeHandle& node_handle)
  : m_node_handle(node_handle),
    m_camera_info_sub(NULL), m_camera_info_got(false), m_debug(false),
    m_visualizer(NULL), m_visualization(false)
{
  createDetectionAlgorithms();
}

// ---------------------------------------------------------------------------

ObjectDetectorProvider::ObjectDetectorProvider(
  const ros::NodeHandle& node_handle, const CameraBridge &camera)
  : m_node_handle(node_handle),
  m_camera_info_sub(NULL), m_camera(camera), m_camera_info_got(true), 
  m_debug(false), m_visualizer(NULL), m_visualization(false)
{
  createDetectionAlgorithms();
}

// ---------------------------------------------------------------------------

inline void ObjectDetectorProvider::createDetectionAlgorithms()
{
  m_algorithms.insert(make_pair("planar", new SurfPlanarDetector));
  m_algorithms.insert(make_pair("3D", new Surf3DDetector));
}

// ---------------------------------------------------------------------------

bool ObjectDetectorProvider::ServiceSearchFor(
		re_vision::SearchFor::Request  &req,
  	re_vision::SearchFor::Response &res )
{
  cv::Mat image = getImage(req);
  
  showRequestInformation(req, image);
  
  if(m_camera_info_got)
    processRequest(req, image, res);
  else
    emptyResponse(req, res);
  
	return true;
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::TopicNewModel
  (const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("New model path: %s", msg->data.c_str());
  learnNewModel(msg->data);
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::TopicCameraInfo
  (const sensor_msgs::CameraInfo::ConstPtr& caminfo)
{
  if(!m_camera_info_got)
  {
    // unsubscribe
    delete m_camera_info_sub;
    m_camera_info_sub = NULL;
    
    /*
    CameraBridge::ImageType imgtype = CameraBridge::BW;
    
    if(m_color_model == "bw") imgtype = CameraBridge::BW;
    else if(m_color_model == "rgb") imgtype = CameraBridge::RGB;
    else if(m_color_model == "bgr") imgtype = CameraBridge::BGR;
    else if(m_color_model == "bayer_bg") imgtype = CameraBridge::BAYER_BG;
    else if(m_color_model == "bayer_gb") imgtype = CameraBridge::BAYER_GB;
    else if(m_color_model == "bayer_rg") imgtype = CameraBridge::BAYER_RG;
    else if(m_color_model == "bayer_gr") imgtype = CameraBridge::BAYER_GR;
    */
    
    int w = caminfo->width;
    int h = caminfo->height;
    float fx = caminfo->K[0];
    float fy = caminfo->K[4];
    float cx = caminfo->K[2];
    float cy = caminfo->K[5];
    float k1, k2, p1, p2;
    
    k1 = caminfo->D[0];
    k2 = caminfo->D[1];
    p1 = caminfo->D[2];
    p2 = caminfo->D[3];

    //m_camera.SetParameters(imgtype, w, h, cx, cy, fx, fy, k1, k2, p1, p2);
    m_camera.SetParameters(w, h, cx, cy, fx, fy, k1, k2, p1, p2);

    ROS_INFO("Camera info got");
    
    ROS_DEBUG("width %d, height %d, "
      "fx %f, fy %f, cx %f, cy %f, k1 %f, k2 %f, p1 %f, p2 %f",
      w, h, fx, fy, cx, cy, k1, k2, p1, p2);
    
    m_camera_info_got = true;
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::init()
{
  m_service = m_node_handle.advertiseService("/re_vision/search_for", 
  	&ObjectDetectorProvider::ServiceSearchFor, this);
    
  m_new_model_sub = m_node_handle.subscribe("/re_vslam/new_model", 
    100, &ObjectDetectorProvider::TopicNewModel, this);

  m_visualization_pub = m_node_handle.advertise<sensor_msgs::Image>
    ("/re_vision/detector_visualization", 100);
  
  if(m_visualizer) delete m_visualizer;
  m_visualizer = new VisualizationManager(m_visualization_pub);

  if(!m_camera_info_got)
  {
    m_camera_info_sub = new ros::Subscriber(
      m_node_handle.subscribe("/camera_info",
        100, &ObjectDetectorProvider::TopicCameraInfo, this));
  }
  
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::SetVisualizationMode(bool onoff)
{
  m_visualization = onoff;
  
  tAlgorithmMap::iterator ait;
  if(m_visualization)
  {
    for(ait = m_algorithms.begin(); ait != m_algorithms.end(); ++ait)
    {
      ait->second->setVisualizationManager(m_visualizer);
    }
  }
  else
  {
    for(ait = m_algorithms.begin(); ait != m_algorithms.end(); ++ait)
    {
      ait->second->setVisualizationManager(NULL);
    }
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::SetDebugMode(bool onoff, 
  const std::string &dir)
{
  tAlgorithmMap::iterator ait;
  for(ait = m_algorithms.begin(); ait != m_algorithms.end(); ++ait)
  {
    ait->second->setDebugMode(onoff, dir);
  }
  m_debug = onoff;
}

// ---------------------------------------------------------------------------

ObjectDetectorProvider::~ObjectDetectorProvider()
{
  delete m_camera_info_sub;
  delete m_visualizer;
  
  tAlgorithmMap::iterator ait;
  for(ait = m_algorithms.begin(); ait != m_algorithms.end(); ++ait)
  {
    delete ait->second;
  }
  
  tModelMap::iterator mit;
  for(mit = m_models.begin(); mit != m_models.end(); ++mit)
  {
    delete mit->second;
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::getValidObjects(const std::vector<std::string> &objects,
	std::vector<std::string>& valid_objects,
	std::vector<re_msgs::DetectedObject> &detections,
	std::vector<re_msgs::DetectedObject *> &pointers) const
{
	// Adds to detections a pointer to an entry in pointers
  detections.clear();
  valid_objects.clear();

  detections.resize(objects.size());
  valid_objects.reserve(objects.size());
  pointers.reserve(objects.size());

  for(unsigned int idx = 0; idx < objects.size(); ++idx)
  {
    // check if the object is valid
    tModelMap::const_iterator pit;
    for(pit = m_models.begin(); pit != m_models.end(); ++pit)
    {
      if(objects[idx] == pit->first) break;
    }
    if(pit != m_models.end())
    {
      valid_objects.push_back(objects[idx]);
      pointers.push_back( &detections[idx] );
    }
    else
    {
      // unknown object
      ROS_WARN("Object \"%s\" unknown", objects[idx].c_str()); 
      detections[idx].points2d.clear();
      detections[idx].points3d.clear();
    }
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::processRequest(
	const re_vision::SearchFor::Request  &req,
	const cv::Mat &image,
	re_vision::SearchFor::Response &res)
{
  ROS_DEBUG("Processing request...");

  cv::Mat _image = image.clone(); // ###

  ros::WallTime t_begin = ros::WallTime::now();
  
  vector<string> valid_objects;
  vector<re_msgs::DetectedObject *> pointers;
  getValidObjects(req.Objects, valid_objects, res.Detections, pointers);
  
  detectObjects(valid_objects, image, req.MaxPointsPerObject, pointers);  
  rectifyDetections(res.Detections, image.cols, image.rows, 
    req.MaxPointsPerObject);
  
#if 0
  if(!res.Detections.empty() && !res.Detections[0].points2d.empty())
  {
    vector<cv::Point3f> points3d;
    for(unsigned int i = 0; i < res.Detections[0].points3d.size(); ++i)
    {
      points3d.push_back(cv::Point3f(
        -res.Detections[0].points3d[i].y,
        -res.Detections[0].points3d[i].z,
        res.Detections[0].points3d[i].x));
    }
    
    cv::Mat cP(points3d);
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    
    vector<cv::Point2f> cam_pixels;
    
    cv::projectPoints(cP, R, t, m_camera.GetIntrinsicParameters(), 
      cv::Mat::zeros(4,1,CV_32F), cam_pixels);
    
    CvScalar color = cvScalar(255, 255, 255);
    for(unsigned int i = 0; i < cam_pixels.size(); ++i)
    {
      cv::circle(_image, cvPoint(cam_pixels[i].x, cam_pixels[i].y), 
        2, color, 1);
    }
    
    cv::imwrite("_debug.png", _image);
    
  }
#endif
  
  ros::WallTime t_end = ros::WallTime::now();
  
  {
    int counter = 0;
    vector<re_msgs::DetectedObject>::const_iterator dit;
    for(dit = res.Detections.begin(); dit != res.Detections.end(); ++dit)
    {
      if(!dit->points3d.empty()) counter++;
    }
    ROS_INFO("%d objects detected", counter);
  }

  ros::WallDuration d = t_end - t_begin;
  ROS_DEBUG("Processing ok. Elapsed time: %f", d.toSec());
}

// ---------------------------------------------------------------------------

inline void  ObjectDetectorProvider::emptyResponse
  (const re_vision::SearchFor::Request &req,
  re_vision::SearchFor::Response &res) const
{
  res.Detections.clear();
	res.Detections.resize(req.Objects.size());
	
	ROS_INFO("Request got, but there is not camera information yet");
}

// ---------------------------------------------------------------------------

cv::Mat ObjectDetectorProvider::getImage(const re_vision::SearchFor::Request &req)
{
  boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr
    (new sensor_msgs::Image(req.Image));

  IplImage *im;
  if(m_camera.usesBayer())
  {
    // m_camera converts the image
    im = m_bridge.imgMsgToCv(ros_img_ptr, "passthrough");
  }
  else
  {
    // ros converts the image
    im = m_bridge.imgMsgToCv(ros_img_ptr, "mono8");
  }
  
  cv::Mat ret = cv::Mat (im).clone ();

  m_camera.ConvertImage(ret);

  return ret;
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::learnNewModel(const std::string &path)
{
  ObjectModel *model = NULL;
  
  try
  {
    model = new ObjectModel(path, true);
    
    pair<tModelMap::iterator, bool> ret =   
      m_models.insert(make_pair( model->Name, model ));
    
    if(!ret.second)
    {
      ROS_INFO("The object %s was already known, updating model...", 
        model->Name.c_str());
      
      delete ret.first->second;
      ret.first->second = model;
    }
  
  } catch(std::string ex)
  {
    delete model;
    ROS_ERROR("Error loading model from directory %s: %s",
      path.c_str(), ex.c_str());
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::removeModel(const std::string &name)
{
  tModelMap::iterator mit = m_models.find(name);
  
  if(mit != m_models.end())
  {
    delete mit->second;
    m_models.erase(mit);
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::detectObjects(
  const std::vector<std::string> &objects,
  const cv::Mat &image, int max_points_per_object,
  std::vector<re_msgs::DetectedObject*> &ret)
{
  // initiate Data for recognition
  // this does not copy the image or the camera
  ObjectDetectorMethod::DetectionData data(image, m_camera); 

  tModelMap::const_iterator mit;
  tAlgorithmMap::const_iterator ait;
  
  std::vector<std::string>::const_iterator oit;
  for(oit = objects.begin(); oit != objects.end(); ++oit)
  {
    mit = m_models.find(*oit);
    
    if(mit != m_models.end())
    {
      ObjectModel &model = *(mit->second);
      
      ait = m_algorithms.find(model.Type);
      if(ait != m_algorithms.end())
      {
        ObjectDetectorMethod *method = ait->second;
        re_msgs::DetectedObject &detection = *ret[oit - objects.begin()];
        
        method->detect(data, model, detection);
        
        // remove points if there are too many
        // (this is now done after rectification)
        //if(max_points_per_object >= 0)
        //  removeSomePoints(detection, max_points_per_object);
        
      }
      else
      {
        ROS_ERROR("Object \"%s\" is of type \"%s\", but there is no algorithm "
          "defined for these objects", oit->c_str(), model.Type.c_str()); 
      }
    }
    else
    {
      // this should not happen since objects already contains valid names
      ROS_WARN("Object \"%s\" unknown", oit->c_str()); 
    }
  }

}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::rectifyDetections(
  vector<re_msgs::DetectedObject> &detections, int W, int H,
  int max_points_per_object) const
{  
  vector<re_msgs::Pixel>::iterator pit;
  vector<re_msgs::DetectedObject>::iterator it;
  for(it = detections.begin(); it != detections.end(); ++it)
  {
    vector<unsigned int> i_remove; // remove those that end up outside
    
    for(pit = it->points2d.begin(); pit != it->points2d.end(); ++pit){
	    m_camera.DistortPoint(pit->x, pit->y);
	    
	    if(pit->x < 0 || pit->x >= W || pit->y < 0 || pit->y >= H)
	    {
	      i_remove.push_back(pit - it->points2d.begin());
	    }
    }
    
    if(!i_remove.empty())
    {
      DUtils::STL::removeIndices(it->points2d, i_remove, false);
      DUtils::STL::removeIndices(it->points3d, i_remove, false);
      DUtils::STL::removeIndices(it->points3d_model, i_remove, false);
    }
    
    if(max_points_per_object >= 0 && 
      (int)it->points2d.size() > max_points_per_object)
    {
      removeSomePoints(*it, max_points_per_object);
    }
  }
}

// ---------------------------------------------------------------------------

void ObjectDetectorProvider::showRequestInformation(
	const re_vision::SearchFor::Request &req,
	const cv::Mat &image) const
{
  static int idx = 0;
  stringstream ss;

  ROS_INFO("Service /re_vision/search_for invoked %d", idx++);

  ss << "Received a " << image.cols << "x" << image.rows 
    << " image and " << req.Objects.size() << " objects: ";

  vector<string>::const_iterator it;
  for(it = req.Objects.begin(); it != req.Objects.end(); ++it) 
    ss << *it << " ";

  ROS_INFO("%s", ss.str().c_str());
}

// ---------------------------------------------------------------------------

/*
typedef vector<unsigned int> *tCell; // < inlier index >

static bool CellsInDescendingOrder(
	const tCell &a, 
	const tCell &b)
{
	return a->size() > b->size();
}
*/

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

void ObjectDetectorProvider::removeSomePoints(
  re_msgs::DetectedObject& detection, int max_points) const
{
  if((int)detection.points2d.size() <= max_points) return;
  
  if(max_points <= 1)
  {
    if(max_points < 0) max_points = 0;
    // trivial case
    detection.points2d.resize(max_points);
    detection.points3d.resize(max_points);
    detection.points3d_model.resize(max_points);
    return;
  }

  // random algorithm
  vector<unsigned int> i_remove;
  i_remove.reserve(detection.points2d.size());
  
  for(unsigned int i = 0; i < detection.points2d.size(); ++i)
  {
    i_remove.push_back(i);
  }
  
  DUtils::Random::SeedRandOnce();
  
  int N = max_points;
  while(N-- > 0)
  {
    int idx = DUtils::Random::RandomInt(0, i_remove.size()-1);
    
    i_remove[idx] = i_remove.back();
    i_remove.pop_back();
  }
  
  // do the removal
  DUtils::STL::removeIndices(detection.points2d, i_remove, false);
  DUtils::STL::removeIndices(detection.points3d, i_remove, false);
  DUtils::STL::removeIndices(detection.points3d_model, i_remove, false);  

  /*
  // convex hull algorithm
  
  // get the points of the convex hull 
  vector<cv::Point2f> p2d_v;
  p2d_v.reserve(detection.points2d.size());
  for(unsigned int i = 0; i < detection.points2d.size(); ++i)
  {
    const re_msgs::Pixel &p = detection.points2d[i];
    p2d_v.push_back(cv::Point2f(p.x, p.y));
  }
  
  cv::Mat p2d(p2d_v, false);
  vector<int> i_hull;
  cv::convexHull(p2d, i_hull);
  
  vector<unsigned int> i_remove;
  i_remove.reserve(detection.points2d.size());
  
  sort(i_hull.begin(), i_hull.end());
  
  // create a vector will all the indices to remove, but those from the
  // convex hull
  if(!i_hull.empty())
  {
    for(int idx = 0; idx < i_hull[0]; ++idx)
    {
      i_remove.push_back(idx);
    }
    for(unsigned int i = 0; i < i_hull.size() - 1; ++i)
    {
      // add from i_hull[i] + 1 to i_hull[i+1]-1
      for(int idx = i_hull[i]+1; idx < i_hull[i+1]; ++idx)
      {
        i_remove.push_back(idx);
      }
    }
    // add from i_hull[last]+1 to N
    for(int idx = i_hull.back()+1; idx < (int)detection.points2d.size(); ++idx)
    {
      i_remove.push_back(idx);
    }
  }
  else
  {
    for(unsigned int idx = 0; idx < detection.points2d.size(); ++idx)
    {
      i_remove.push_back(idx);
    }
  }
  
  if(!i_hull.empty())
  {
    if((int)i_hull.size() < max_points)
    {
      // pick random points to save
      DUtils::Random::SeedRandOnce();
      int N = max_points - i_hull.size();
      while(N-- > 0)
      {
        int idx = DUtils::Random::RandomInt(0, i_remove.size());
        
        // save that index by removing the idx-th entry from i_remove
        i_remove[idx] = i_remove.back();
        i_remove.pop_back();
      }
    } // i_hull.size() < max_points
    else if((int)i_hull.size() > max_points)
    {
      // remove some points from the hull
      
      // remove those whose edges are shorter
      vector<float> len;
      len.reserve(i_hull.size());
      
      int N = i_hull.size() - max_points;
      while(N-- > 0)
      {
        len.resize(0);
        len.resize(i_hull.size(), 0);
        
        int idx_prev = i_hull.back();
        int idx_cur = i_hull[0];
        int idx_post = i_hull[1];
        
        for(unsigned int i = 0; i < i_hull.size(); ++i)
        {
          // len of edges
          const re_msgs::Pixel& prev = detection.points2d[idx_prev];
          const re_msgs::Pixel& cur = detection.points2d[idx_cur];
          const re_msgs::Pixel& post = detection.points2d[idx_post];
          
          len[i] = (cur.x - prev.x)*(cur.x - prev.x) + 
            (cur.y - prev.y)*(cur.y - prev.y) +
            (cur.x - post.x)*(cur.x - post.x) +
            (cur.y - post.y)*(cur.y - post.y);
          
          //
          //const cv::Point2f &prev = p2d_v[idx_prev];
          //const cv::Point2f &cur = p2d_v[idx_cur];
          //const cv::Point2f &post = p2d_v[idx_post];
          //
          //const float Ax = cur.x - prev.x;
          //const float Ay = cur.y - prev.y;
          //const float Bx = post.x - prev.x;
          //const float By = post.y - prev.y;
          //
          //// |Ax * By - Ay * Bx| == | A x B | == area of parallelogram 
          //// == 2 * area of the triangle depicted by points prev, cur and post
          //len[i] = fabs( Ax * By - Ay * Bx ) / 2
          //  + sqrt(Ax*Ax - Ay*Ay) + sqrt(Bx*Bx - By*By);
          //// len[i] = area + length of edges
          //
          
          idx_prev = idx_cur;
          idx_cur = idx_post;
          if(i == i_hull.size() - 1)
            idx_post = i_hull[0];
          else
            idx_post = i_hull[i+1];
        }
        
        // get the point with shortest edges
        vector<unsigned int> remove_now;
        remove_now.push_back(
          min_element(len.begin(), len.end()) - len.begin());
        
        // flag to remove it later
        i_remove.push_back( i_hull[remove_now.back()] );
        
        // and remove it now from the convex hull
        DUtils::STL::removeIndices(i_hull, remove_now, true);
      }
      
    } // i_hull.size() > max_points
  } // i_hull.empty()
  
  // do the removal
  DUtils::STL::removeIndices(detection.points2d, i_remove, false);
  DUtils::STL::removeIndices(detection.points3d, i_remove, false);
  DUtils::STL::removeIndices(detection.points3d_model, i_remove, false);

  */

  /* // grid algorithm

  // the remaining points should be as much spread as possible
  float minx, miny, maxx, maxy; // bounding box of points in the image
  minx = miny = 1e5;
  maxx = maxy = 0;
  
  vector<re_msgs::Pixel>::const_iterator pit;
  for(pit = detection.points2d.begin(); pit != detection.points2d.end(); ++pit)
  {
    if(pit->x < minx) minx = pit->x;
    if(pit->x > maxx) maxx = pit->x;
    if(pit->y < miny) miny = pit->y;
    if(pit->y > maxy) maxy = pit->y;
  }

  const float width = maxx - minx + 1;
  const float height = maxy - miny + 1;

	const float ratioWH = width / height;
	
	// inflate the number of desired points to avoid rejecting many points
	// when only a few of them are desired
	const int DesiredPoints = max_points * 3;
	
	// size of grid in cells
	int GridW = (int)cvCeil(cvSqrt(DesiredPoints * ratioWH));
	int GridH = (int)cvCeil((float)DesiredPoints / GridW);
	const int NCells = GridW * GridH;
	
	// size of cell in pixels
	const float CellW = width / GridW;
	const float CellH = height / GridH;
	
	// cells stored by rows
	vector<tCell> cells(NCells); // each tCell contains inlier indices
	for(int i = 0; i < NCells; ++i){
		cells[i] = new vector<unsigned int>;
	}
	
	// fill the grid
  for(unsigned int i = 0; i < detection.points2d.size(); ++i)
  {
    int gridx = int((detection.points2d[i].x - minx) / CellW);
    int gridy = int((detection.points2d[i].y - miny) / CellH);
    int i_cell = gridy * GridW + gridx;

    cells[i_cell]->push_back(i); // inlier index
  }
  
  // sort the cells in descending order of size
  sort(cells.begin(), cells.end(), CellsInDescendingOrder);
  
  // remove points until getting max_points points
  int last_cell = 0;
  int existing_points = detection.points2d.size();
  bool goon = true;

  vector<unsigned int> i_remove; // indices of points2d, points3d, etc to remove

  while(goon)
  {
    unsigned int this_level_points = cells[last_cell]->size();

    while(last_cell < NCells && cells[last_cell]->size() == this_level_points)
      last_cell++;
    last_cell--;

    // check how many points we should remove 
    int this_level_cells = last_cell + 1;
    int remove_total = existing_points - max_points;
    int remove_from_level;

    if(last_cell < NCells-1)
    {
      int should_remove = this_level_points - cells[last_cell+1]->size();
      should_remove *= this_level_cells;

      if(should_remove < remove_total)
        remove_from_level = should_remove;
      else
        remove_from_level = remove_total;

    }else
      remove_from_level = remove_total;
		
    int remove_from_cell = 
      (int)cvCeil((double)remove_from_level / (double)this_level_cells);

    for(int i_cell = 0; i_cell <= last_cell && remove_total > 0; ++i_cell)
    {
      int remove_now = (remove_from_cell < remove_total ? 
        remove_from_cell : remove_total);

      if(remove_now > 0)
      {
        // get the last indices of *cells[i_cell]
        vector<unsigned int> &cell = *cells[i_cell];
        
        if((int)cell.size() <= remove_now)
        {
          i_remove.insert(i_remove.end(), cell.begin(), cell.end());
          cell.clear();
        }
        else
        {
          i_remove.insert(i_remove.end(), 
            cell.begin() + (cell.size() - remove_now), cell.end());
          cell.resize(cell.size() - remove_now);
        }

        existing_points -= remove_now;
        remove_total -= remove_now;
      }
    }

    goon = existing_points > 0 && remove_total > 0;
  }

  // release aux data
  for(int i = 0; i < NCells; ++i) delete cells[i];
  
  // do the actual removal
  DUtils::STL::removeIndices(detection.points2d, i_remove, false);
  DUtils::STL::removeIndices(detection.points3d, i_remove, false);
  DUtils::STL::removeIndices(detection.points3d_model, i_remove, false);
  */
}

// ---------------------------------------------------------------------------

