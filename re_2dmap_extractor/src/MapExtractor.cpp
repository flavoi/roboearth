/** \file MapExtractor.cpp
 * \brief Tool for extracting 2d maps from an 3d Octomap
 *
 * MapExtractor.cpp extracts 2d maps from an 3d Octomap in two different modes.
 * On the one hand, a simple 2d slice can be taken out of the Octomap and used
 * as a map for localization. On the other hand, a 3d slice of the Octomap can
 * be chosen to be projected down to a 2d map, which may be used for
 * navigation. The latter mode is intended to be used to create 2d maps, that
 * take obstacles into account, which are not visible to a base laser scanner.
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

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <iomanip>
#include "MapExtractor.h"

using namespace std;
using namespace roboearth;

MapExtractor::MapExtractor(const std::string& filename) :
	m_gridmap(NULL), m_octoMap(NULL), m_res(0.025), m_treeDepth(0) {

	cout << "Loading 3d octomap '" << filename.c_str() << "':" << endl << "  ";
	cout << setprecision(3);

	m_octoMap = new octomap::OcTree(m_res);
	if (m_octoMap->readBinary(filename)) {

		m_treeDepth = m_octoMap->getTreeDepth();
		m_res = m_octoMap->getResolution();
		m_gridmap = new OccupancyGrid;
		m_gridmap->resolution = m_res;

	} else {
		cout << endl;
		exit(-1);
	}

}

MapExtractor::~MapExtractor() {

	delete m_octoMap;
	delete m_gridmap;

}

bool MapExtractor::isSpeckleNode(const octomap::OcTreeKey& nKey) const {
	octomap::OcTreeKey key;
	bool neighborFound = false;
	for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]) {
		for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]) {
			for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]) {
				if (key != nKey) {
					octomap::OcTreeNode* node = m_octoMap->search(key);
					if (node && m_octoMap->isNodeOccupied(node)) {
						neighborFound = true;
					}
				}
			}
		}
	}

	return !neighborFound;
}

bool MapExtractor::calc2dSlice(double occupancyMinZ, double occupancyMaxZ,
		bool filterSpeckles, double minSizeX, double minSizeY) {

	// This tree implementation currently has a maximum depth of 16
	// nodes. For this reason, coordinates values have to be, e.g.,
	// below +/- 327.68 meters (2^15) at a maximum resolution of 0.01m.

	double minX, minY, minZ, maxX, maxY, maxZ;
	m_octoMap->getMetricMin(minX, minY, minZ);
	m_octoMap->getMetricMax(maxX, maxY, maxZ);

	cout << "  min - x: " << minX << " y: " << minY << " z: " << minZ << endl;
	cout << "  max - x: " << maxX << " y: " << maxY << " z: " << maxZ << endl;

	octomap::point3d minPt(minX, minY, minZ);
	octomap::point3d maxPt(maxX, maxY, maxZ);
	octomap::OcTreeKey minKey, maxKey, curKey;

	if (!m_octoMap->genKey(minPt, minKey)) {
		cout << "  Creating min octree key (x: " << minPt.x() << ", y: "
				<< minPt.y() << ", z: " << minPt.z() << ") failed." << endl;
		return false;
	}
	if (!m_octoMap->genKey(maxPt, maxKey)) {
		cout << "  Creating max octree key (x: " << maxPt.x() << ", y: "
				<< maxPt.y() << ", z: " << maxPt.z() << ") failed." << endl;
		return false;
	}

	cout << "  min key - x: " << minKey[0] << " y: " << minKey[1] << " z: "
			<< minKey[2] << endl;
	cout << "  max key - x: " << maxKey[0] << " y: " << maxKey[1] << " z: "
			<< maxKey[2] << endl;

	// pad to min size
	double halfPaddedX = 0.5 * minSizeX;
	double halfPaddedY = 0.5 * minSizeY;
	minX = std::min(minX, -halfPaddedX);
	maxX = std::max(maxX, halfPaddedX);
	minY = std::min(minY, -halfPaddedY);
	maxY = std::max(maxY, halfPaddedY);
	minPt = octomap::point3d(minX, minY, minZ);
	maxPt = octomap::point3d(maxX, maxY, maxZ);
	octomap::OcTreeKey paddedMinKey, paddedMaxKey;

	if (!m_octoMap->genKey(minPt, paddedMinKey)) {
		cout << "  Creating padded min octree key (x: " << minPt.x() << ", y: "
				<< minPt.y() << ", z: " << minPt.z() << ") failed." << endl;
		return false;
	}
	if (!m_octoMap->genKey(maxPt, paddedMaxKey)) {
		cout << "  Creating padded max octree key (x: " << maxPt.x() << ", y: "
				<< maxPt.y() << ", z: " << maxPt.z() << ") failed." << endl;
		return false;
	}

	if (paddedMinKey[0] != minKey[0] || paddedMinKey[1] != minKey[1]) {
		cout << "  padded min key - x: " << paddedMinKey[0] << " y: "
				<< paddedMinKey[1] << " z: " << paddedMinKey[2] << endl;
		cout << "  padded max key - x: " << paddedMaxKey[0] << " y: "
				<< paddedMaxKey[1] << " z: " << paddedMaxKey[2] << endl;

		assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);
	}

	m_gridmap->width = paddedMaxKey[0] - paddedMinKey[0] + 1;
	m_gridmap->height = paddedMaxKey[1] - paddedMinKey[1] + 1;
	int mapOriginX = minKey[0] - paddedMinKey[0];
	int mapOriginY = minKey[1] - paddedMinKey[1];

	octomap::point3d origin;
	m_octoMap->genCoords(paddedMinKey, m_treeDepth, origin);
	m_gridmap->orientation.x = origin.x() - m_res * 0.5;
	m_gridmap->orientation.y = origin.y() - m_res * 0.5;
	m_gridmap->orientation.yaw = 0; //TODO calc yaw for grid map

	// allocate space for grid map, init with -1 (occupancy unknown)
	m_gridmap->data.resize(m_gridmap->width * m_gridmap->height, -1);

	// run through all leafs of the tree
	for (octomap::OcTree::leaf_iterator it = m_octoMap->begin(), end =
			m_octoMap->end(); it != end; ++it) {

		if (m_octoMap->isNodeOccupied(*it)) { // node is occupied

			double z = it.getZ();
			if (z > occupancyMinZ && z < occupancyMaxZ) {

				double size = it.getSize();
				double x = it.getX();
				double y = it.getY();
				octomap::OcTreeKey nKey = it.getKey();

				// optionally ignore single occupied nodes without direct neighbors
				if (filterSpeckles && (it.getDepth() == m_treeDepth + 1)
						&& isSpeckleNode(nKey)) {
					cout << "Ignoring single occupied node - x: " << x
							<< " y: " << y << " z: " << z << endl;
					continue;
				}

				if (it.getDepth() == m_treeDepth) {
					octomap::OcTreeKey nKey = it.getKey();
					int i = nKey[0] - paddedMinKey[0];
					int j = nKey[1] - paddedMinKey[1];
					m_gridmap->data[m_gridmap->width * j + i] = 100; //occupied
				} else {
					int intSize = 1 << (m_treeDepth - it.getDepth());
					octomap::OcTreeKey minKey = it.getIndexKey();
					for (int dx = 0; dx < intSize; dx++) {
						int i = minKey[0] + dx - paddedMinKey[0];
						for (int dy = 0; dy < intSize; dy++) {
							int j = minKey[1] + dy - paddedMinKey[1];
							m_gridmap->data[m_gridmap->width * j + i] = 100;
						}
					}
				}

			}

		} else { // node is free

			if (it.getDepth() == m_treeDepth) {

				octomap::OcTreeKey nKey = it.getKey();
				int i = nKey[0] - paddedMinKey[0];
				int j = nKey[1] - paddedMinKey[1];
				if (m_gridmap->data[m_gridmap->width * j + i] == -1) {
					m_gridmap->data[m_gridmap->width * j + i] = 0; // free
				}

			} else {

				int intSize = 1 << (m_treeDepth - it.getDepth());
				octomap::OcTreeKey minKey = it.getIndexKey();
				for (int dx = 0; dx < intSize; dx++) {
					int i = minKey[0] + dx - paddedMinKey[0];
					for (int dy = 0; dy < intSize; dy++) {
						int j = minKey[1] + dy - paddedMinKey[1];
						if (m_gridmap->data[m_gridmap->width * j + i] == -1) {
							m_gridmap->data[m_gridmap->width * j + i] = 0;
						}
					}
				}

			}

		}

	}

	return true;

}

bool MapExtractor::save2dMap(const std::string& mapName, double z,
		bool filterSpeckles, double minSizeX, double minSizeY) {

	return save2dMap(mapName, z - m_res, z + m_res, filterSpeckles, minSizeX,
			minSizeY);

}

bool MapExtractor::save2dMap(const std::string& mapName, double minZ,
		double maxZ, bool filterSpeckles, double minSizeX, double minSizeY) {

	if (!calc2dSlice(minZ, maxZ, filterSpeckles, minSizeX, minSizeY)) {
		return false;
	}

	cout << "  size: " << m_gridmap->width << " x " << m_gridmap->height
			<< ", resolution: " << m_gridmap->resolution << " m/pixel" << endl;

	string mapFilename = mapName + ".pgm";
	FILE* outMap = fopen(mapFilename.c_str(), "w");
	if (!outMap) {
		cout << "couldn't save map to '" << mapFilename.c_str() << "'" << endl;
		return false;
	}

	fprintf(outMap, "P5\n# CREATOR: MapExtractor.cpp %.3f m/pix\n%d %d\n255\n",
			m_gridmap->resolution, m_gridmap->width, m_gridmap->height);

	for (unsigned int y = 0; y < m_gridmap->height; y++) {
		for (unsigned int x = 0; x < m_gridmap->width; x++) {
			unsigned int i = x + (m_gridmap->height - y - 1) * m_gridmap->width;
			if (m_gridmap->data[i] == 0) {
				fputc(254, outMap);
			} else if (m_gridmap->data[i] == +100) {
				fputc(000, outMap);
			} else {
				fputc(205, outMap);
			}
		}
	}

	fclose(outMap);

	string metaFilename = mapName + ".yaml";
	FILE* outMeta = fopen(metaFilename.c_str(), "w");
	if (!outMeta) {
		cout << "couldn't save meta data to '" << metaFilename.c_str() << "'"
				<< endl;
		return false;
	}

	std::string::size_type filename_begin = mapFilename.find_last_of('/');
	if (filename_begin == std::string::npos) {
		filename_begin = 0;
	} else {
		filename_begin++;
	}
	std::string::size_type filename_length = mapFilename.length() - filename_begin;
	mapFilename = mapFilename.substr(filename_begin, filename_length);

	fprintf(
			outMeta,
			"image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
			mapFilename.c_str(), m_gridmap->resolution,
			m_gridmap->orientation.x, m_gridmap->orientation.y,
			m_gridmap->orientation.yaw);

	fclose(outMeta);

	return true;

}
