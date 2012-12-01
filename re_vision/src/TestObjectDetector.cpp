/** \file TestObjectDetector.cpp
 * \brief Invokes the SearchFor service for testing
 *
 * App that calls the SearchFor service for debuggin purposes
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
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include "MetaFile.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/Image.h>
#include <boost/shared_ptr.hpp>

#include "re_vision/SearchFor.h"

#include <cstdlib>
#include <string>
#include <iomanip>

#include <boost/shared_ptr.hpp>

using namespace std;

// - - -

class ImageGrabber
{
public:
  ImageGrabber(){}
  ~ImageGrabber(){}
  void topicImageRawGot(const sensor_msgs::Image::ConstPtr& msg);
};

// - - - 

cv::Mat loadSomeImage(int idx);
void createRequest(re_vision::SearchFor &srv, const cv::Mat &image);
void showResponse(const re_vision::SearchFor &srv);
void storeResponse(const re_vision::SearchFor &srv, 
				   const cv::Mat &image, int idx);

// - - -

//static vector<ParsedObjectInformation> m_object_list;
static vector<string> m_object_list;
static sensor_msgs::CvBridge m_bridge;
//static ParsedOptions m_options;

static ros::NodeHandle *m_n;
static ros::Subscriber m_camera_sub;
static ros::ServiceClient m_client;
static ImageGrabber m_grabber;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int main(int argc, char **argv)
{
  sleep(1);
  
	ros::init(argc, argv, "TestObjectDetector");
	
	m_n = new ros::NodeHandle;

  m_client = m_n->serviceClient<re_vision::SearchFor>("/re_vision/search_for");

  m_camera_sub = m_n->subscribe("/image_raw", 
    1, &ImageGrabber::topicImageRawGot, &m_grabber);

	//parseObjectFile("objects3d.txt", m_object_list, m_options);
	//if(m_options.Use3DObjects) loadNames(m_object_list);
	
	std::string object = "bottle1";
	if(argc >= 2) object = argv[1];
	
	if(object == "all")
	{
	  m_object_list.push_back("van");
	  /*
	  m_object_list.push_back("cabinet1");
	  m_object_list.push_back("bottle1");
	  m_object_list.push_back("bed1");
	  */
	}
	else
	{
	  m_object_list.push_back(object); 
	}
	
	stringstream ss;
  for(unsigned int k = 0; k < m_object_list.size(); ++k)
  {
    ss << m_object_list[k] << " ";
  }
	
	if(false)
  	ROS_INFO("There are %d available 3d objects: %s", m_object_list.size(),
  	  ss.str().c_str());
  else
    ROS_INFO("There are %d availale planar objects: %s", m_object_list.size(),
      ss.str().c_str());
	
	re_vision::SearchFor srv;
	const int N = 0;
	//const int N = 41;  // Eind Jan 11 test: 13 (6 for model images)
	// oso: 182, fragoneta_mesa: 41, orbit_mesa: 24
	// workshop: 20; rss_light: 177; rss_dark: 194; poster_2: 206; poster_1: 178
	// rss_cerca: 1
  
	vector<int> counters(m_object_list.size(), 0);
	
	for(int i = 0; i < N; ++i){
		cv::Mat image = loadSomeImage(i);

		createRequest(srv, image);
		
		ss.str("");
		for(unsigned int k = 0; k < srv.request.Objects.size(); ++k)
		{
		  ss << srv.request.Objects[k] << " ";
		}
		ROS_INFO("Sending request to find object(s): %s", ss.str().c_str());

		if (m_client.call(srv))
		{
			ROS_INFO("Request %d made ok", i);
			showResponse(srv);
			
			//if(srv.response.state.state == re_msgs::State::READY){
				storeResponse(srv, image, i);

				for(unsigned int j = 0; j < m_object_list.size(); j++)
					if(!srv.response.Detections[j].points2d.empty()) counters[j]++;
			//}
			
		}else{
			ROS_ERROR("Failed to call service re_vision::SearchFor (%d)", i);
			return 1;
		}
	}
  
  cout << "Counters:" << endl;
  for(unsigned int j = 0; j < m_object_list.size(); j++){
	cout << "Object " << j << ": " << counters[j] << endl;
  }

  ros::spin();
  
  delete m_n;

  return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void ImageGrabber::topicImageRawGot(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_WARN("/image_raw got");
  
  static int i = -1;
  i++;
  
  re_vision::SearchFor srv;
	
	IplImage * img_ipl = 
	  m_bridge.imgMsgToCv(msg, "passthrough");
	cv::Mat image = cv::Mat(img_ipl).clone();

	createRequest(srv, image);
	
	stringstream ss;
	for(unsigned int k = 0; k < srv.request.Objects.size(); ++k)
	{
	  ss << srv.request.Objects[k] << " ";
	}
	ROS_INFO("Sending request to find object(s): %s", ss.str().c_str());

	if (m_client.call(srv))
	{
		ROS_INFO("Request %d made ok", i);
		showResponse(srv);
		
		//if(srv.response.state.state == re_msgs::State::READY){
			storeResponse(srv, image, i);

		//}
		
	}else{
		ROS_ERROR("Failed to call service re_vision::SearchFor (%d)", i);
	}
	
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void createRequest(re_vision::SearchFor &srv, const cv::Mat &image)
{
	//IplImage *img = cvCloneImage(&IplImage(image));
	IplImage img(image);
  
	try{
		sensor_msgs::Image::Ptr ros_img_ptr = m_bridge.cvToImgMsg(&img, "passthrough");
		srv.request.Image = sensor_msgs::Image(*ros_img_ptr);
    
	}catch (sensor_msgs::CvBridgeException error){
		ROS_ERROR("error in createRequest");
	}

	srv.request.Objects.clear();
	for(unsigned int j = 0; j < m_object_list.size(); ++j)
		srv.request.Objects.push_back(m_object_list[j]);
  
	srv.request.MaxPointsPerObject = 3;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void showResponse(const re_vision::SearchFor &srv)
{
	const vector<re_msgs::DetectedObject> &D = srv.response.Detections;
	
	stringstream ss;
  
	//if(srv.response.state.state != re_msgs::State::READY){
	if(0){
		ss << "ObjectDetector is not ready yet";
	}else{
		ss << "Response got: ";
		
		if(!D.empty()) ss << endl;
		
		vector<re_msgs::DetectedObject>::const_iterator it;
		vector<re_msgs::Pixel>::const_iterator pit;
		vector<geometry_msgs::Point>::const_iterator pit3d;
		for(it = D.begin(); it != D.end(); ++it){
			ss << " + Object " << it - D.begin() << " ";
			
			if(it->points2d.empty())
			  ss << "(not found)";
			else
			  ss << "(found)";
			  
			ss << ": " 
				<< "position: X=" << it->pose.position.x 
				<< ", Y=" << it->pose.position.y
				<< ", Z=" << it->pose.position.z
				<< ", quaternion: "
				<< it->pose.orientation.x << " "
				<< it->pose.orientation.y << " "
				<< it->pose.orientation.z << " "
				<< it->pose.orientation.w << ". "
				<< "There are " << it->points2d.size() << " points detected";
			
			if(!it->points2d.empty()){
				ss << ":" << endl << "   ";
				for(pit = it->points2d.begin(); pit != it->points2d.end(); ++pit){
					pit3d = it->points3d.begin() + (pit - it->points2d.begin());
					
					ss << "(" << pit->x << ", " << pit->y << " => "
						<< pit3d->x << ", " << pit3d->y << ", " << pit3d->z << "), ";
				}
			}
			
			ss << endl;
		}
	}
  
	ROS_INFO("%s", ss.str().c_str());  	
	
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

inline int getImIdx(int idx){
  return idx + 0;
}

cv::Mat loadSomeImage(int idx)
{
	const int im_idx = getImIdx(idx);
	stringstream ss;
	//ss << "images/sequence_2/image_" << setw(3) << setfill('0') << im_idx << "-L.bmp";
	//ss << "images/pike/box-" << setw(10) << setfill('0') << im_idx << ".ppm";
	//ss << "images/rss_dark/rss-" << setw(3) << setfill('0') << im_idx << ".png";
	//ss << "images/poster_1/image_" << setw(4) << setfill('0') << im_idx << "-L.bmp";
	//ss << "images/poster_2/image_" << setw(4) << setfill('0') << im_idx << "-L.bmp";
	//ss << "images/workshop_wide/image_" << setw(3) << setfill('0') << im_idx << ".png";
	//ss << "images/cerca/image_" << setw(3) << setfill('0') << im_idx << ".png";
	//ss << "images/rss/rss-grande-claro.png";
	
	//ss << "models/test_orbit/face_" << setw(3) << setfill('0') << im_idx << ".png";
	//ss << "images/orbit_mesa/subset/frame" << setw(2) << setfill('0') << im_idx << ".jpg";
	//ss << "images/fragoneta_mesa/frame" << setw(4) << setfill('0') << im_idx << ".jpg";
	//ss << "images/secuenciaOso/frame" << setw(4) << setfill('0') << im_idx << ".jpg";
	
	//ss << "../src/extra/image_000.png";
	//ss << "../src/extra/gazebo1.jpg";
	//ss << "../src/extra/frameBottle.jpg";
	
	//ss << "../src/extra/lab0-rect.png";
	//ss << "../src/extra/test/frame" << setw(4) << setfill('0') << im_idx << ".jpg"; // ####
	//ss << "../src/extra/formodel/frame" << setw(4) << setfill('0') << im_idx+2 << ".jpg"; // ####
	
	ss << "/home/dorian/Universidad/Proyectos/RoboEarth/SvnUnizar/ObjectDetectorOld/images/fragoneta_mesa/frame"
	  << setw(4) << setfill('0') << im_idx << ".jpg";
	
	cv::Mat m = cv::imread(ss.str(), 0);

	if(m.cols == 0)
		ROS_ERROR("Could not find image file: %s", ss.str().c_str()); 
	
	return m;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void storeResponse(const re_vision::SearchFor &srv, 
				   const cv::Mat &image, int idx)
{
  const int im_idx = getImIdx(idx);
  stringstream ss;
  
  //ss << "test_results/image_" << setw(3) << setfill('0') << im_idx << "-L.png";
  ss << "TestObjectDetector_result.png";
  
  cv::Mat im;
  cv::cvtColor(image, im, CV_GRAY2RGB, 3);
  
  CvScalar colors[] = {
	cvScalar(255, 0, 0),
	cvScalar(0, 255, 0)
  };
  
  // draw the detected points
  vector<re_msgs::DetectedObject>::const_iterator it;
	for(it = srv.response.Detections.begin(); 
	  it != srv.response.Detections.end(); ++it)
	{
	  ROS_DEBUG("Image %d, object %d, points %d", im_idx, 
				it -srv.response.Detections.begin(),
				it->points2d.size());
				
	  CvScalar color = colors[it - srv.response.Detections.begin()];
	  
	  vector<re_msgs::Pixel>::const_iterator pit;
	  for(pit = it->points2d.begin(); pit != it->points2d.end(); ++pit){
		  cv::circle(im, cvPoint(pit->x, pit->y), 5, color, 1);
	  }
	}
  
  cv::imwrite(ss.str(), im);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

