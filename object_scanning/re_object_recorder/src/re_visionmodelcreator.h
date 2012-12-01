/** \file re_visionmodelcreator.h
 * \brief Creates object models for WP1's object recognition
 *
 * Tries to create object models from colored 3D point clouds that can be used by re_vision.
 *
 * This file is part of the RoboEarth ROS re_object_recorder package.
 *
 * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by by <a href="mailto:andreas.koch@ipvs.uni-stuttgart.de">Andreas Koch</a>, University of Stuttgart
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    <UL>
 *     <LI> Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     <LI> Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     <LI> Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *    </UL>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Andreas Koch
 * \author Daniel Di Marco
 * \version 1.0
 * \date 2011
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
***********************************************/

#ifndef RE_VISIONMODELCREATOR_H
#define RE_VISIONMODELCREATOR_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QDir>
#include <QString>

// Enable this define to estimate the normals and store them in ply files
// (can take a long time for bigger clouds)
//#define ESTIMATE_NORMALS

/**
 * Creates recognition models compatible with re_vision from a set of dense point clouds.
 **/
class re_visionModelCreator
{
public:
    /**
     * Constructor.
     * @param pointclouds different views from the model as dense point clouds
     **/
    re_visionModelCreator(const std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > >& pointclouds);

    /**
     * Store a recognition model compatible to re_vision to a given directory.
     * @param parent_dir target directory
     * @param merged_pcl the merged point cloud
     * @param modelname the name of the model
     **/
    void createModel(const QDir& parent_dir, const pcl::PointCloud<pcl::PointXYZRGB>& merged_pcl, const QString& modelname);

    /// utility function to store a PCL point cloud as PLY file
    static void saveAsPLY(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcl, const QString& filename);
    /// creates a meta file compatible to re_vision
    static void createMetaFile(QDir model_dir, QString modelname, QString modeltype, size_t face_count, double scale);

protected:
    const std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > >& pcls;
};

#endif // RE_VISIONMODELCREATOR_H
