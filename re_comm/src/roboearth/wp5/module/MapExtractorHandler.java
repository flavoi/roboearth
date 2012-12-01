/* \file MapExtractorHandler.java
 * \brief 2d map extraction handler
 *
 * The 2d map extractor handler provides the callbacks for ROS services 
 * related to requesting 2d maps from RoboEarth.
 * 
 * This file is part of the RoboEarth ROS re_comm package.
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
package roboearth.wp5.module;

import java.util.ArrayList;

import org.semanticweb.owlapi.model.OWLOntology;

import roboearth.wp5.conn.REInterface;
import roboearth.wp5.owl.OWLIO;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.re_srvs.srv.Request2DMap;
import ros.pkg.re_srvs.srv.RequestProj2DMap;

public class MapExtractorHandler extends AbstractHandler {

	public MapExtractorHandler(Ros ros, NodeHandle n) throws RosException {
		super(ros, n);
		
		this.n.advertiseService("/re_comm/request_2d_map", new Request2DMap(), new Request2DMapCallback());
		this.n.advertiseService("/re_comm/request_proj_2d_map", new RequestProj2DMap(), new RequestProj2DMapCallback());
		
		ros.logInfo("Module 'MapExtractorHandler' loaded.");
	}

	/**
	 * 
	 * The callback class for the /re_comm/request_2d_map ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class Request2DMapCallback implements ServiceServer.Callback<Request2DMap.Request, Request2DMap.Response> {

		@Override
		public Request2DMap.Response call(Request2DMap.Request req) {

			Request2DMap.Response res = new Request2DMap.Response();
			res.success = false;

			REInterface con = getREConnection(guestInterfaceKey);
			OWLOntology srdl = OWLIO.loadOntologyFromString(req.srdl);
			ArrayList<byte[]> map = con.request2dMap(req.envUID, srdl, req.baseScannerLink, req.targetMapName);

			res.success = (map == null || map.size() != 2 || 
					map.get(0) == null || map.get(1) == null ? false:true);
			if (res.success) {
				res.map.name = req.targetMapName+".pgm";
				res.map.data = map.get(0);
				res.meta.name = req.targetMapName+".yaml";
				res.meta.data = map.get(1);
				ros.logInfo("Request2dMap (UID: " + req.envUID + "): Done");
			} else {
				ros.logInfo("Request2dMap (UID: " + req.envUID + "): Failed");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/request_proj_2d_map ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class RequestProj2DMapCallback implements ServiceServer.Callback<RequestProj2DMap.Request, RequestProj2DMap.Response> {

		@Override
		public RequestProj2DMap.Response call(RequestProj2DMap.Request req) {

			RequestProj2DMap.Response res = new RequestProj2DMap.Response();
			res.success = false;

			REInterface con = getREConnection(guestInterfaceKey);
			
			ArrayList<byte[]> map = con.requestProjected2dMap(req.envUID, req.minZ, req.maxZ, req.targetMapName);

			res.success = (map == null || map.size() != 2 || 
					map.get(0) == null || map.get(1) == null ? false:true);
			if (res.success) {
				res.map.name = req.targetMapName+".pgm";
				res.map.data = map.get(0);
				res.meta.name = req.targetMapName+".yaml";
				res.meta.data = map.get(1);
				ros.logInfo("RequestProj2dMap (UID: " + req.envUID + ", minZ: " + req.minZ + ",maxZ: " + req.maxZ + "): Done");
			} else {
				ros.logInfo("RequestProj2dMap (UID: " + req.envUID + ", minZ: " + req.minZ + ",maxZ: " + req.maxZ + "): Failed");
			}

			return res;

		}

	}
	
}
