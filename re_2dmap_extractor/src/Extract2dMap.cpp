/** \file Extract2dMap.cpp
 * \brief Command line parser for the 2d map extractor tool
 *
 * Extract2dMap.cpp parses command line arguments and initiates the 2d map
 * extraction process.
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

#include <stdexcept>
#include <getopt.h>
#include "MapExtractor.h"

using namespace std;
using namespace roboearth;

#define USAGE "\nUsage: \n" \
		"This program takes a 3d slice out of an Octomap and performs\n" \
		"a 2d projection (if parameters min_z and max_z are given) or\n" \
		"extracts a 2d map at height min_z (if max_z is missing)\n" \
		"and saves the result as an image. Related meta data \n" \
		"get stored in a seperate yaml file.\n" \
		"\nOptions:\n" \
		"-h, --help \tshow this description\n" \
		"-o, --octomap \t3d octomap to read and work on\n" \
		"-z, --min_z \tminimum height to consider for 2d projection\n" \
		"\t\t/ height to extract 2d map at\n" \
		"-Z, --max_z \tmaximum height to consider for 2d projection\n" \
		"-n, --name \tfilename without extension used for storing \n" \
		"\t\tresulting 2d image (.pgm) and meta data (.yaml)\n" \
		"\nExamples:\n" \
		"extract2dMap -z 0.303 -o hospital_room.bt -n hr_303\n" \
		"extract2dMap -z 0.2 -Z 1.5 -o hospital_room.bt -n hr_2to15\n" \
		"extract2dMap --min_z 0.2 --max_z 1.5 --octomap hospital_room.bt " \
		"--name hr_2to15\n"

int main(int argc, char** argv) {

	cout << endl;

	string inOctomap("");
	string outMapname("");

	double minZ = -std::numeric_limits<double>::max();
	double maxZ = std::numeric_limits<double>::max();

	char* ePtr;
	bool usage = false;

	static const struct option long_options[] = {
			{ "help", no_argument, 0, 'h' },
			{ "min_z", required_argument, 0, 'z' },
			{ "max_z", required_argument, 0, 'Z' },
			{ "octomap", required_argument, 0, 'o' },
			{ "name", required_argument, 0, 'n' },
			{ 0,0,0,0 }
	};

	bool minZset = false;
	bool maxZset = false;
	
	/* parse options */
	while (optind < argc) {

		int index = -1;
		struct option * opt = 0;

		int result = getopt_long(argc, argv, "ho:z:Z:n:", long_options, &index);
		if (result == -1) {
			break;
		}

		switch (result) {
		case 'h':
			usage = true;
			break;
		case 'o':
			inOctomap = optarg;
			break;
		case 'z':
			minZ = strtod(optarg, &ePtr);
			minZset = true;
			if (*ePtr != '\0') {
				cout << "ERROR: value for 'min_z' could not be parsed: " << optarg << endl;
				usage = true;
			}
			break;
		case 'Z':
			maxZ = strtod(optarg, &ePtr);
			maxZset = true;
			if (*ePtr != '\0') {
				cout << "ERROR: value for 'max_z' could not be parsed: " << optarg << endl;
				usage = true;
			}
			break;
		case 'n':
			outMapname = optarg;
			break;
		default:
			return 1;
		}
	}
	/* print all other parameters */
	while (optind < argc) {
		cout << "INFO: ignoring other parameter: " << argv[optind++] << endl;
	}

	if (inOctomap.empty()) {
		cout << "ERROR: no 3d octomap specified (-o / --octomap)" << endl;
		usage = true;
	}
	if (outMapname.empty()) {
		cout << "ERROR: name for output map not specified (-n / --name)" << endl;
		usage = true;
	}
	if (!minZset) {
		cout << "ERROR: min_z has to be specified (-z / --min_z)" << endl;
		usage = true;
	}
	if (minZ > maxZ) {
		cout << "ERROR: min_z is larger than max_z (min_z:" << minZ << ", max_z:" << maxZ << ")" << endl;
		usage = true;
	}

	if (usage) {

		cout << USAGE << endl;
		return 1;

	} else {

		try {

			MapExtractor mapEx(inOctomap); // load 3d octomap

			cout << endl << "Exporting 2d map '" << outMapname.c_str() << "'";
			cout << endl;

			bool success = false;
			if (!maxZset) {
			  success = mapEx.save2dMap(outMapname, minZ, true); // save 2dmap
			} else {
			  success = mapEx.save2dMap(outMapname, minZ, maxZ, true); // save projected 2dmap
			}
			  			
			if (success) {
				cout << "done." << endl;
			} else {
				cout << "failed." << endl;
			}

		} catch (runtime_error& e) {
			cout << e.what() << endl << endl;
			return 2;
		}

	}

	cout << endl;

}
