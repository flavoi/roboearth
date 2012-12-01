/* \file EnvironmentHandler.java
 * \brief Environment handler
 *
 * The environment handler provides the callbacks for ROS services related to environments.
 * 
 * This file is part of the RoboEarth ROS re_comm package.
 * 
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2010 by 
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
 * \date 2010
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
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
import ros.pkg.re_msgs.msg.StringArray;
import ros.pkg.re_srvs.srv.DelEnvironment;
import ros.pkg.re_srvs.srv.GetEnvironment;
import ros.pkg.re_srvs.srv.QueryEnvironments;
import ros.pkg.re_srvs.srv.SearchEnvironments;
import ros.pkg.re_srvs.srv.SetEnvironment;
import ros.pkg.re_srvs.srv.UpdateEnvironment;

public class EnvironmentHandler extends AbstractHandler {

	public EnvironmentHandler(Ros ros, NodeHandle n) throws RosException {

		super(ros, n);

		this.n.advertiseService("/re_comm/get_environment", new GetEnvironment(), new GetEnvironmentCallback());
		this.n.advertiseService("/re_comm/set_environment", new SetEnvironment(), new SetEnvironmentCallback());
		this.n.advertiseService("/re_comm/del_environment", new DelEnvironment(), new DelEnvironmentCallback());
		this.n.advertiseService("/re_comm/update_environment", new UpdateEnvironment(), new UpdateEnvironmentCallback());
		this.n.advertiseService("/re_comm/search_environments", new SearchEnvironments(), new SearchEnvironmentsCallback());
		this.n.advertiseService("/re_comm/query_environments", new QueryEnvironments(), new QueryEnvironmentsCallback());

		ros.logInfo("Module 'EnvironmentHandler' loaded.");

	}

	/**
	 * 
	 * The callback class for the /re_comm/get_environment ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class GetEnvironmentCallback implements ServiceServer.Callback<GetEnvironment.Request, GetEnvironment.Response> {

		@Override
		public GetEnvironment.Response call(GetEnvironment.Request req) {

			GetEnvironment.Response res = new GetEnvironment.Response();
			
			REInterface con = getREConnection(guestInterfaceKey);
			ArrayList<String> outFilenames = new ArrayList<String>();
			ArrayList<String> outFileURLs = new ArrayList<String>();
			String ont = con.requestEnvironment(req.environmentUID, outFilenames, outFileURLs);
			
			if (ont != null) {
				res.environment = ont;
				res.filenames = outFilenames;
				res.fileURLs = outFileURLs;
				res.success = true;
				ros.logInfo("GetEnvironment (UID: " + req.environmentUID + "): Done");
			} else {
				res.success = false;
				ros.logInfo("GetEnvironment (UID: " + req.environmentUID + "): Failed");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/set_environment ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SetEnvironmentCallback implements ServiceServer.Callback<SetEnvironment.Request, SetEnvironment.Response> {

		@Override
		public SetEnvironment.Response call(SetEnvironment.Request req) {

			SetEnvironment.Response res = new SetEnvironment.Response();
			res.success = false;
			
			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				OWLOntology ont = OWLIO.loadOntologyFromString(req.environment);
				
				ArrayList<String> filenames = new ArrayList<String>();
				ArrayList<byte[]> dataArray = new ArrayList<byte[]>();
				for (ros.pkg.re_msgs.msg.File file : req.files) {
					filenames.add(file.name);
					dataArray.add(file.data);
				}
				
				res.success = con.submitEnvironment(ont, req.cls, req.id, req.description, dataArray, filenames);
				if (res.success) {
					ros.logInfo("SetEnvironment (UID: " + req.cls+"."+req.id + "): Done");
				} else {
					ros.logInfo("SetEnvironment (UID: " + req.cls+"."+req.id + "): Failed");
				}

			} else {
				ros.logError("SetEnvironment (UID: " + req.cls+"."+req.id + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/del_environment ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class DelEnvironmentCallback implements ServiceServer.Callback<DelEnvironment.Request, DelEnvironment.Response> {

		@Override
		public DelEnvironment.Response call(DelEnvironment.Request req) {

			DelEnvironment.Response res = new DelEnvironment.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.deleteEnvironment(req.environmentUID);
				if (res.success) {
					ros.logInfo("DelEnvironment (UID: " + req.environmentUID + "): Done");
				} else {
					ros.logInfo("DelEnvironment (UID: " + req.environmentUID + "): Failed");
				}

			} else {
				ros.logError("DelEnvironment (UID: " + req.environmentUID + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/update_environment ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class UpdateEnvironmentCallback implements ServiceServer.Callback<UpdateEnvironment.Request, UpdateEnvironment.Response> {

		@Override
		public UpdateEnvironment.Response call(UpdateEnvironment.Request req) {

			UpdateEnvironment.Response res = new UpdateEnvironment.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				OWLOntology ont = OWLIO.loadOntologyFromString(req.environment);
				res.success = con.updateEnvironment(req.uid, ont, req.description);
				if (res.success) {
					ros.logInfo("UpdateEnvironment (UID: " + req.uid + "): Done");
				} else {
					ros.logInfo("UpdateEnvironment (UID: " + req.uid + "): Failed");
				}

			} else {
				ros.logError("UpdateEnvironment (UID: " + req.uid + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/search_environments ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SearchEnvironmentsCallback implements ServiceServer.Callback<SearchEnvironments.Request, SearchEnvironments.Response> {

		@Override
		public SearchEnvironments.Response call(SearchEnvironments.Request req) {

			SearchEnvironments.Response res = new SearchEnvironments.Response();
			res.success = false;

			REInterface con = getREConnection(guestInterfaceKey);
			ArrayList<String> uids = new ArrayList<String>();
			ArrayList<ArrayList<String>> filenames = new ArrayList<ArrayList<String>>();
			ArrayList<ArrayList<String>> urls = new ArrayList<ArrayList<String>>();
			
			String[] onts = con.searchEnvironments(req.searchID, uids, filenames, urls);
			if (onts != null) {
				for (String ont : onts) {
					StringArray saNames = new StringArray();
					saNames.list = filenames.remove(0);
					res.filenames.add(saNames);
					
					StringArray saURLs = new StringArray();
					saURLs.list = urls.remove(0);
					res.fileURLs.add(saURLs);
					
					res.environments.add(ont);
				}
				res.uids = uids;
				res.success = true;
				ros.logInfo("SearchEnvironments (search: " + req.searchID + "): Done");				
			} else {
				res.success = false;
				ros.logInfo("SearchEnvironments (search: " + req.searchID + "): Failed");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/query_environments ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class QueryEnvironmentsCallback implements ServiceServer.Callback<QueryEnvironments.Request, QueryEnvironments.Response> {

		@Override
		public QueryEnvironments.Response call(QueryEnvironments.Request req) {

			QueryEnvironments.Response res = new QueryEnvironments.Response();
			res.result = "";

			REInterface con = getREConnection(guestInterfaceKey);
			String result = con.queryEnvironmentDB(req.query);
			if (result != null) {
				res.result = result;
				ros.logInfo("QueryEnvironment (query:\n" + req.query + "): Done");
			} else {
				ros.logInfo("QueryEnvironment (query:\n" + req.query + "): Failed");
			}

			return res;

		}

	}
	
}
