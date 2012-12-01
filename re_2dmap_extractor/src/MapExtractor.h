/** \file MapExtractor.h
 * \brief Header file for 2d map extractor tool
 *
 * This file is part of the RoboEarth ROS re_2dmap_extractor package.
 *
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the
 * European Union Seventh Framework Programme FP7/2007-2013
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by
 * <a href=" mailto:perzylo@cs.tum.edu">Alexander Perzylo</a>
 * Technische Universitaet Muenchen
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
 * \author Alexander Perzylo
 * \version 1.0
 * \date 2011
 */

#ifndef MAPEXTRACTOR_H
#define MAPEXTRACTOR_H

#include <octomap/OcTree.h>

namespace roboearth {

struct Orientation {
	double x;
	double y;
	double yaw;
};

struct OccupancyGrid {
	unsigned int width;
	unsigned int height;
	double resolution;
	Orientation orientation;
	std::vector<char> data;
};

class MapExtractor {

public:
	MapExtractor(const std::string& octomapFilename);
	virtual ~MapExtractor();
	bool save2dMap(const std::string& mapName, double z, bool filterSpeckles,
			double minSizeX = 0, double minSizeY = 0);
	bool save2dMap(const std::string& mapName, double minZ, double maxZ,
			bool filterSpeckles, double minSizeX = 0, double minSizeY = 0);

protected:
	void resetOctomap();
	bool isSpeckleNode(const octomap::OcTreeKey& nKey) const;
	bool calc2dSlice(double occupancyMinZ, double occupancyMaxZ,
			bool filterSpeckles, double minSizeX, double minSizeY);

	OccupancyGrid *m_gridmap;
	octomap::OcTree *m_octoMap;
	double m_res;
	unsigned int m_treeDepth;
};

}

#endif
