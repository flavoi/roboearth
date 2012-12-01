/** \file recognitionmodel.h
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

#ifndef RECOGNITIONMODEL_H
#define RECOGNITIONMODEL_H

#include <cv.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Enable this define to show debug visualization
//#define DEBUG_VIS

#ifdef DEBUG_VIS
#include <highgui.h>
#endif

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;

/**
 * An object aspect corresponds to a single view of the model. It stores a set of
 * SURF keypoints and descriptors along with 3D information for these.
 * Recognition models consist of multiple aspects. When detecting an object, we go
 * through all object aspects and try to find the best match.
 **/
class ObjectAspect {
public:
    ObjectAspect();

    /**
     * Calculates the SURF keypoints and descriptors for this aspect and determines their 3D coordinates.
     * @param image 2D camera image
     * @param pointcloud dense point cloud for extracting 3D coordinates
     **/
    int calculate(const cv::Mat& image, const PointCloud::Ptr& pointcloud);

    /**
     * Loads an ObjectAspect from a dense .pcd file.
     * @param filename filename to a dense .pcd
     **/
    void fromFile(const std::string& filename);
    void plotKeypoints(cv::Mat &image);
    /**
     * Finds a set of correspondences between this and another object aspect and gives a list of correspondences.
     * @param other the other ObjectAspect to compare
     * @param ptpairs output parameter, where each cv::Point2i contains (index of this ObjectAspect's keypoint, index of other's keypoint)
     **/
    void findCorrespondences(const ObjectAspect& other, std::vector<cv::Point2i>& ptpairs);

    /**
     * Finds the nearest neighbor of a given SURF descriptor.
     * @return index of the nearest neighbor, or -1 if none is found
     * @param vec SURF descriptor
     * @param model_keypoints keypoints of the ObjectAspect
     * @param model_descriptors descriptors of the ObjectAspect
     * @param imageMap2D3D map from 2D points to 3D points
     **/
    static int naiveNearestNeighbor(const float* vec, const std::vector<cv::KeyPoint>& model_keypoints,
                             const float* model_descriptors, const std::map<int,int>& imageMap2D3D);

    /**
     * Calculates the distance between 2 SURF descriptors.
     * @return euclidean distance
     * @param d1 first descriptor
     * @param d2 second descriptor
     * @param best lowest distance found until now
     * @param length length of descriptors (has to be a multiple of 4)
     **/
    static double compareSURFDescriptors(const float* d1, const float* d2, double best, int length);

    /**
     * Tries to match 2 aspects and calculates the resulting transformation.
     * @return 1 for success, DBL_MAX for failure
     * @param other the ObjectAspect to compare with
     * @param output parameter, gives the resulting transformation
     **/
    double matchAspects(ObjectAspect& other, Eigen::Matrix4f& transform);

    /// 2D keypoints
    std::vector<cv::KeyPoint> keypoints;
    /// SURF descriptors
    std::vector<float> descriptors;
    /// maps 2D keypoints to 3D coordinates (indices from keypoints to indices from keypoints3D)
    std::map<int,int> map2D3D;
    /// inverse to map2D3D
    std::map<int,int> map2D3Dinv;
    /// 3D keypoints
    PointCloud::Ptr keypoints3D;

    /// 2D input image
    cv::Mat image;
    /// input pointcloud
    PointCloud::Ptr points;
    ObjectAspect* match;
};

/**
 * Object recognition model.
 * Consists of multiple ObjectAspects and a model name.
 **/
class RecognitionModel
{
public:
    RecognitionModel();

    /**
     * Load a recognition model from the given directory.
     * @return true for success, false for failure
     * @param path the path to load the model from
     **/
    bool loadFromPath(const std::string& path);

    std::vector<ObjectAspect*> aspects;
    /**
     * Tries to match a object aspect with this recognition model and
     * calculates the resulting transformation.
     * @return true if successful, false otherwise
     * @param other ObjectAspect to compare
     * @param model2scene output parameter, the resulting transformation
     **/
    bool matchAspects(ObjectAspect& other, Eigen::Matrix4f& model2scene);
    std::string model_name;
};

#endif // RECOGNITIONMODEL_H
