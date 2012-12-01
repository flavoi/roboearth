/** \file recognitionmodel.cpp
 * \brief 3D point cloud recognition model
 *
 * This file is part of the RoboEarth ROS re_kinect_object_detector package.
 *
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by <a href="mailto:andreas.koch@ipvs.uni-stuttgart.de">Andreas Koch</a>, University of Stuttgart
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \author Andreas Koch
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 ***********************************************/

#include "recognitionmodel.h"

#include <fstream>
#include <cstdio>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/nonfree/features2d.hpp>
#include <cv_bridge/CvBridge.h>
#include <pcl/io/pcd_io.h>
#include <boost/version.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <tinyxml.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace Utils3D {
    /// calculate the euclidean distance between 2 3D points
    double distPoints(const cv::Point3f& p1, const cv::Point3f& p2)
    {
        cv::Point3f d = p1 - p2;
        return sqrt(d.x*d.x + d.y*d.y + d.z*d.z );
    }
}

ObjectAspect::ObjectAspect() {
    keypoints3D = PointCloud::Ptr(new PointCloud);
    points = PointCloud::Ptr(new PointCloud);
}

int ObjectAspect::calculate(const cv::Mat &image, const PointCloud::Ptr &pointcloud) {

    cv::Mat gray,timg;

    image.copyTo(this->image);
    this->points = pointcloud;

#if 1
    cv::cvtColor(image,gray,CV_BGR2HLS);
    //gray = planes[2]-planes[1];
    std::vector<cv::Mat> planes;
    cv::split(gray,planes);

    gray = planes[1];
    cv::merge(&planes[0],(size_t)3,timg);
    cv::cvtColor(timg,timg,CV_HSV2BGR);
#else
    timg = image;
#endif

#ifdef DEBUG_VIS
    cv::cvtColor(gray,timg,CV_GRAY2BGR);
#endif

    const int HessianThreshold = 500;
    cv::SURF bug(HessianThreshold);
    bug(gray,255*cv::Mat::ones(gray.rows,gray.cols,CV_8U),keypoints,descriptors);


    for(size_t i = 0; i < keypoints.size(); i++) {
        cv::KeyPoint kp = keypoints[i];
        PointType pt3d = pointcloud->at(kp.pt.x,kp.pt.y);
        pt3d.rgb = kp.response;
        if(!isnan(pt3d.x)/*&&(kp.response/1000>3)*/) {
            keypoints3D->points.push_back(pt3d);
            map2D3D.insert(std::make_pair<int,int>(i,keypoints3D->points.size()-1));
            map2D3Dinv.insert(std::make_pair<int,int>(keypoints3D->points.size()-1,i));
        }
    }

#ifdef DEBUG_VIS
    plotKeypoints(timg);

    cv::namedWindow("surf");
    cv::imshow("surf",timg);
#endif

    return 0;
}

double ObjectAspect::matchAspects(ObjectAspect &other, Eigen::Matrix4f &transform) {
    std::vector<int> o_pcl2, t_pcl2;
    std::vector<cv::Point2i> correspondences;
    findCorrespondences(other, correspondences);

#ifdef DEBUG_VIS
    cv::Mat3b image_stack(std::max(this->image.size().height,other.image.size().height), this->image.size().width+other.image.size().width );

    cv::Mat3b img1 = image_stack(cv::Rect(0,0,this->image.size().width,this->image.size().height));
    cv::Mat3b img2 = image_stack(cv::Rect(this->image.size().width,0,other.image.size().width,other.image.size().height));

    this->image.copyTo(img1);
    other.image.copyTo(img2);

    this->plotKeypoints(img1);
    other.plotKeypoints(img2);
#endif

    for (size_t i = 0; i < correspondences.size(); i++) {

        int kpidx = correspondences[i].x;
        int lkpidx = correspondences[i].y;

        //cv::line(image, keypoints.at(kpidx ).pt,other.keypoints.at(lkpidx).pt,cv::Scalar(205,0,255),3);

        if ( map2D3D.find(kpidx) != map2D3D.end() && other.map2D3D.find(lkpidx) != other.map2D3D.end() ) {
            t_pcl2.push_back(kpidx);
            o_pcl2.push_back(lkpidx);
#ifdef DEBUG_VIS
            cv::line(image_stack,this->keypoints[kpidx].pt,other.keypoints[lkpidx].pt+cv::Point2f(this->image.size().width,0),cv::Scalar(255,0,0));
#endif
        }
    }

//    ROS_INFO("found %d correspondences.",correspondences.size());
//    ROS_INFO("found %d 3D correspondences.",t_pcl2.size());

    // distances that differ by <= EPSILON are considered equal
    const float EPSILON = 0.001;
    // distance threshold between 2 keypoints to be considered equal
    const float DISTANCE_THRESHOLD = 0.02;

    PointCloud o_pcl, t_pcl;
    if (t_pcl2.size()>=3) {
        for (size_t i = 0; i < t_pcl2.size()-2; i ++ ){
            PointType npt1 = this->keypoints3D->points[this->map2D3D[t_pcl2.at(i)]];
            PointType lpt1 = other.keypoints3D->points[other.map2D3D[o_pcl2.at(i)]];
            cv::Point3f np1 = cv::Point3f(npt1.x,npt1.y,npt1.z);
            cv::Point3f lp1 = cv::Point3f(lpt1.x,lpt1.y,lpt1.z);
            for (size_t j = i+1; j < t_pcl2.size()-1; j++) {
                PointType npt2 = this->keypoints3D->points[this->map2D3D[t_pcl2.at(j)]];
                PointType lpt2 = other.keypoints3D->points[other.map2D3D[o_pcl2.at(j)]];
                cv::Point3f np2 = cv::Point3f(npt2.x,npt2.y,npt2.z);
                cv::Point3f lp2 = cv::Point3f(lpt2.x,lpt2.y,lpt2.z);
                if ( Utils3D::distPoints(np1,np2) < DISTANCE_THRESHOLD || fabs(Utils3D::distPoints(np1,np2) - Utils3D::distPoints(lp1,lp2)) > EPSILON )
                    continue;
                for (size_t k = j+1; k < t_pcl2.size(); k++) {
                    PointType npt3 = this->keypoints3D->points[this->map2D3D[t_pcl2.at(k)]];
                    PointType lpt3 = other.keypoints3D->points[other.map2D3D[o_pcl2.at(k)]];
                    cv::Point3f np3 = cv::Point3f(npt3.x,npt3.y,npt3.z);
                    cv::Point3f lp3 = cv::Point3f(lpt3.x,lpt3.y,lpt3.z);
                    if ( Utils3D::distPoints(np1,np3) < DISTANCE_THRESHOLD ||  fabs(Utils3D::distPoints(np1,np3) - Utils3D::distPoints(lp1,lp3)) > EPSILON )
                        continue;

                    if (Utils3D::distPoints(np3,np2) < DISTANCE_THRESHOLD ||  fabs(Utils3D::distPoints(np3,np2) - Utils3D::distPoints(lp3,lp2)) > EPSILON )
                        continue;

                    cv::Point3f v1 = np3-np2;
                    cv::Point3f v2 = np1-np2;
                    cv::Point3f v3 = np3-np1;

                    double angle = acos(v1.ddot(v2)/sqrt(v1.ddot(v1))/sqrt(v2.ddot(v2)))/CV_PI*180.;
                    v1 = -1*v1;
                    double angle2 = acos(v1.ddot(v3)/sqrt(v1.ddot(v1))/sqrt(v3.ddot(v3)))/CV_PI*180.;

//                    cerr << "angles: " << angle << ", " << angle2 << endl;

                    // check whether the 3 vectors are linearly independent
                    const double ANGLE_THRESH = 30;
                    if ((fabs(angle)<ANGLE_THRESH) || (fabs(angle2)<ANGLE_THRESH) ||(fabs(angle)>180-ANGLE_THRESH)||(fabs(angle2)>180-ANGLE_THRESH))
                        continue;

//                    cerr << "distances: " <<
//                            Utils3D::distPoints(&np1,&np3) <<"\t"
//                         << Utils3D::distPoints(&np3,&np2)<<
//                            "\t" << Utils3D::distPoints(&np1,&np2) << " i,j,k: " << i << ","<< j << "," << k <<  endl;
#ifdef DEBUG_VIS
                    cv::circle(image_stack, this->keypoints[t_pcl2.at(i)].pt, 3, cv::Scalar(255,255,0),-1);
                    cv::circle(image_stack, this->keypoints[t_pcl2.at(j)].pt, 3, cv::Scalar(255,255,0),-1);
                    cv::circle(image_stack, this->keypoints[t_pcl2.at(k)].pt, 3, cv::Scalar(255,255,0),-1);

                    cv::circle(image_stack, other.keypoints[o_pcl2.at(i)].pt+cv::Point2f(this->image.size().width,0), 3, cv::Scalar(255,255,0),-1);
                    cv::circle(image_stack, other.keypoints[o_pcl2.at(j)].pt+cv::Point2f(this->image.size().width,0), 3, cv::Scalar(255,255,0),-1);
                    cv::circle(image_stack, other.keypoints[o_pcl2.at(k)].pt+cv::Point2f(this->image.size().width,0), 3, cv::Scalar(255,255,0),-1);

                    cv::line(image_stack,this->keypoints[t_pcl2.at(i)].pt,other.keypoints[o_pcl2.at(i)].pt+cv::Point2f(this->image.size().width,0),cv::Scalar(0,255,255),2);
                    cv::line(image_stack,this->keypoints[t_pcl2.at(j)].pt,other.keypoints[o_pcl2.at(j)].pt+cv::Point2f(this->image.size().width,0),cv::Scalar(0,255,255),2);
                    cv::line(image_stack,this->keypoints[t_pcl2.at(k)].pt,other.keypoints[o_pcl2.at(k)].pt+cv::Point2f(this->image.size().width,0),cv::Scalar(0,255,255),2);
#endif
                    o_pcl.push_back(lpt1);
                    o_pcl.push_back(lpt2);
                    o_pcl.push_back(lpt3);

                    t_pcl.push_back(npt1);
                    t_pcl.push_back(npt2);
                    t_pcl.push_back(npt3);
                    break;
                }
                if (t_pcl.size() != 0)
                    break;
            }
            if (t_pcl.size() != 0)
                break;
        }
    }


    if(t_pcl.points.size()==3) {
        // get transformation
        Eigen::Matrix4f t;
        pcl::Correspondences corr;
        pcl::registration::TransformationEstimationSVD<PointType, PointType> estimator;
        estimator.estimateRigidTransformation( t_pcl,o_pcl, t );
        // pcl::estimateRigidTransformationSVD( t_pcl,o_pcl, t ); Mauro
#ifdef DEBUG_VIS
    cv::namedWindow("correspondences");
    cv::imshow("correspondences",image_stack);
#endif
        transform = t;
        return 1;

    }
    return DBL_MAX;
}

int ObjectAspect::naiveNearestNeighbor(const float* vec,
                                       const std::vector<cv::KeyPoint>& model_keypoints,
                                       const float* model_descriptors, const std::map<int,int>& imageMap2D3D) {
    // length of descriptor
    const int LENGTH = 64;
    int neighbor = -1;
    double d = 0.;
    double dist1 = DBL_MAX, dist2 = DBL_MAX;

    for(size_t i = 0; i < model_keypoints.size(); i++ ) {
        if(imageMap2D3D.find(i)==imageMap2D3D.end())
            continue;

        const float* mvec = &model_descriptors[i*64];
        d = compareSURFDescriptors( vec, mvec, dist2, LENGTH );
        if( d < dist1 ) {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}

double
ObjectAspect::compareSURFDescriptors(const float* d1, const float* d2, double best, int length) {
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 ) {
        double t0 = d1[i  ] - d2[i  ];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }

    return total_cost;
}

void ObjectAspect::findCorrespondences(const ObjectAspect& other, std::vector<cv::Point2i>& ptpairs) {
    const std::vector<cv::KeyPoint>& imageKeypoints = other.keypoints;
    const std::vector<float>& imageDescriptors = other.descriptors;
    const std::map<int,int>& imageMap2D3D = other.map2D3D;
    ptpairs.clear();

    for(size_t i = 0; i < keypoints.size(); i++) {
        if(map2D3D.find(i)==map2D3D.end())
            continue;

        const float* mvec = &descriptors[i*64];

        int nearest_neighbor = naiveNearestNeighbor(mvec,
                                                    imageKeypoints,
                                                    &imageDescriptors[0] ,
                                                    imageMap2D3D);
        if (nearest_neighbor >= 0) {
            ptpairs.push_back(cv::Point2i(i,nearest_neighbor));
        }
    }
}


void ObjectAspect::plotKeypoints(cv::Mat &image) {
    const int radius = 1, thickness = 2;
    const cv::Scalar color(0,0,0,0);
    for(size_t i = 0;i<keypoints.size();++i) {
        cv::KeyPoint kp = keypoints[i];
        cv::circle(image,kp.pt,kp.response/1000,cv::Scalar(255,0,0,0));
        cv::circle(image,kp.pt,radius,color,thickness);
        if (map2D3D.find(i) != map2D3D.end())
            cv::circle(image,kp.pt,kp.response/1000,cv::Scalar(0,255,0,0));
    }
}

/**
 * Load a pointcloud from a pcd file.
 * Checks whether the .pcd has padding after the header, and removes it it if needed.
 * (binary PCD files in versions < 1.0 required padding, PCL >= version 1.0 does not)
 * @param filename pcd file to load
 * @param cloud pointer to which the loaded cloud is written
 * @return 0 for success, != 0 otherwise
 **/
int read_binary_PCL(const std::string& filename, PointCloud::Ptr cloud) {
    std::ifstream in_file(filename.c_str());
    std::string token("DATA binary\n");

    const size_t BUFFER_SIZE = 4096;
    char buffer[BUFFER_SIZE];
    in_file.read(buffer, BUFFER_SIZE);
    if (in_file.gcount() < BUFFER_SIZE) {
        std::cerr << "could not read whole file" << std::endl;
        return -1;
    }

    bool found_binary = false;
    bool null_after_header = false;
    size_t matched_chars = 0;
    size_t header_length = 0;
    for(size_t i=0; i<BUFFER_SIZE; i++) {
        if (buffer[i] == token[matched_chars]) {
            matched_chars++;
        } else
            matched_chars = 0;

        if (matched_chars == token.length()) {
            found_binary = true;

            header_length = i;
            size_t num_nulls = 0;
            for(size_t j=header_length+1; j<BUFFER_SIZE; j++) {
                if (buffer[j] == '\0')
                    num_nulls++;
                else
                    break;
            }
            null_after_header = num_nulls > 10;
            if (null_after_header) {
                std::cout << "null bytes found after header, converting from old PCD binary format" << std::endl;
            }
            break;
        }
    }

    std::string tmp_filename;
    int temp_file_fd;
    if (null_after_header) {
        char tmp_name[] = "/tmp/pcd_file_XXXXXX";

        temp_file_fd = mkstemp(tmp_name);
        if (temp_file_fd < 0) {
            std::cerr << "could not create temporary file" << std::endl;
            return -1;
        }
        tmp_filename = tmp_name;

        std::ofstream tmp_file(tmp_filename.c_str());
        tmp_file.write(buffer, header_length+1);

        in_file.seekg(4096, std::ios::beg);
        while(in_file.good()) {
            in_file.read(buffer, BUFFER_SIZE);
            tmp_file.write(buffer, in_file.gcount());
        }
        in_file.close();
        tmp_file.close();
        pcl::io::loadPCDFile(tmp_filename, *cloud);
        remove(tmp_filename.c_str());
    } else {
        in_file.close();
        // just load it
        pcl::io::loadPCDFile(filename, *cloud);
    }

    return 0;
}

void ObjectAspect::fromFile(const std::string &filename) {
    PointCloud::Ptr cloud  = PointCloud::Ptr(new PointCloud);
    read_binary_PCL(filename, cloud);
    sensor_msgs::ImagePtr image_(new sensor_msgs::Image);
    pcl::toROSMsg (*cloud, *image_);
    sensor_msgs::CvBridge bridge;
    IplImage* capture_ = bridge.imgMsgToCv(image_,std::string("bgr8"));
    ROS_INFO("loading model aspect: %s",filename.c_str());
    cv::Mat image = cv::Mat(capture_,true);

    calculate(image,cloud);
    points = cloud;
    // TODO: Load the view point to object base transform from the pointclouds viewpoint;
}

RecognitionModel::RecognitionModel()
{
}

/// load a re_vision style meta file
bool loadMetaFile(const std::string& file_name, std::string& model_name, std::string& model_type, int& face_count) {
    TiXmlDocument doc(file_name);
    bool loaded = doc.LoadFile();
    if (!loaded)
        return false;

    TiXmlHandle hdoc(&doc);
    TiXmlHandle model_handle = hdoc.FirstChildElement("model");
    TiXmlElement *model_element = model_handle.Element();
    if (model_element == NULL)
        return false;
    TiXmlElement *name_element = model_handle.FirstChildElement("name").Element();
    if (name_element == NULL)
        return false;
    model_name = name_element->GetText();
    TiXmlElement *type_element = model_handle.FirstChildElement("type").Element();
    if (type_element == NULL)
        return false;
    model_type = type_element->GetText();
    TiXmlElement *faces_element = model_handle.FirstChildElement("faces").Element();
    if (faces_element == NULL)
        return false;
    std::string faces_str = faces_element->GetText();
    face_count = boost::lexical_cast<int>(faces_str);

    return true;
}


/// Helper function to convert boost path to c strings.
const char *boost_path_to_cstr(const boost::filesystem::path& path) {
#if BOOST_VERSION >= 104600
    return path.c_str();
#else
    return path.native_file_string().c_str();
#endif
}

bool RecognitionModel::loadFromPath(const std::string &path) {
    boost::filesystem::path dir(path);

    if(!boost::filesystem::is_directory(dir)) {
         ROS_ERROR("Not a directory: %s", boost_path_to_cstr(dir));
         return false;
    }

    boost::filesystem::path meta_path(dir / "meta.xml");
    if (!boost::filesystem::exists(meta_path)) {
        ROS_ERROR("No meta.xml file found in %s", boost_path_to_cstr(dir));
        return false;
    }

    int face_count = 0;
    std::string model_type = "";
    bool loaded = loadMetaFile(boost_path_to_cstr(meta_path), model_name, model_type, face_count);

    if (!loaded || model_name.empty() || (face_count == 0) || (model_type != "kinect_pcl")) {
        ROS_ERROR("invalid meta.xml in %s", boost_path_to_cstr(dir));
        return false;
    }

    for(boost::filesystem::directory_iterator it(dir); it!=boost::filesystem::directory_iterator(); ++it)
    {
        boost::filesystem::path pref = *it;
        const static std::string FACE_PREFIX = "dense_face_";

#if BOOST_VERSION >= 104600
        if ((pref.filename().string().substr(0, FACE_PREFIX.length()) == FACE_PREFIX) && (pref.extension() == ".pcd")) {
#else
        if ((pref.filename().substr(0, FACE_PREFIX.length()) == FACE_PREFIX) && (pref.extension() == ".pcd")) {
#endif
            aspects.push_back(new ObjectAspect());
            ObjectAspect* aspect = aspects.back();
            aspect->fromFile(pref.string());
            if (aspects.size() > static_cast<size_t>(face_count)) {
                ROS_INFO("found more faces than specified in meta.xml. please check %s", boost_path_to_cstr(meta_path));
                break;
            }
        }
    }

    if (aspects.size() < static_cast<size_t>(face_count)) {
        ROS_ERROR("meta.xml specified %i faces, but only %i were found.", face_count, aspects.size());
        return false;
    }
    if (aspects.empty()) {
        ROS_ERROR("no model files in path %s", path.c_str());
        return false;
    }
    return true;
}


bool RecognitionModel::matchAspects(ObjectAspect &other, Eigen::Matrix4f &model2scene) {
    double minerror = DBL_MAX;

    for (size_t i = 0; i < aspects.size(); i ++) {
        ObjectAspect* model_aspect = aspects[i];
        Eigen::Matrix4f trafo;
        double error = model_aspect->matchAspects(other,trafo);
        if (error < minerror) {
            model2scene = trafo;
            other.match = model_aspect;
            minerror = error;
            ROS_INFO("HIT %d, minerror: %f",(int)i, minerror);
        }
    }

    return minerror != DBL_MAX;
}
