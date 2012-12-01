/* \file ObjectModelHandler.java
 * \brief Object model handler
 *
 * The object model handler provides the callbacks for ROS services related to object models.
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
import ros.pkg.re_srvs.srv.DelObject;
import ros.pkg.re_srvs.srv.GetObject;
import ros.pkg.re_srvs.srv.QueryObjects;
import ros.pkg.re_srvs.srv.SearchObjects;
import ros.pkg.re_srvs.srv.SetObject;
import ros.pkg.re_srvs.srv.UpdateObject;

public class ObjectModelHandler extends AbstractHandler {

	/**
	 * Constructor. Advertises the provided ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public ObjectModelHandler(Ros ros, NodeHandle n) throws RosException {

		super(ros, n);

		this.n.advertiseService("/re_comm/get_object", new GetObject(), new GetObjectCallback());
		this.n.advertiseService("/re_comm/set_object", new SetObject(), new SetObjectCallback());
		this.n.advertiseService("/re_comm/del_object", new DelObject(), new DelObjectCallback());
		this.n.advertiseService("/re_comm/update_object", new UpdateObject(), new UpdateObjectCallback());
		this.n.advertiseService("/re_comm/search_objects", new SearchObjects(), new SearchObjectsCallback());
		this.n.advertiseService("/re_comm/query_objects", new QueryObjects(), new QueryObjectsCallback());

		ros.logInfo("Module 'ObjectHandler' loaded.");

	}


	public static GetObject.Response getObject(GetObject.Request req) {
		
		GetObject.Response res = new GetObject.Response();
		
		REInterface con = getREConnection(guestInterfaceKey);
		ArrayList<String> outFilenames = new ArrayList<String>();
		ArrayList<String> outFileURLs = new ArrayList<String>();
		String ont = con.requestObject(req.objectUID, outFilenames, outFileURLs);
		
		if (ont != null) {
			res.object = ont;
			res.filenames = outFilenames;
			res.fileURLs = outFileURLs;
			res.success = true;
		} else {
			res.success = false;
		}
		
		return res;
		
	}
	
	public static SetObject.Response setObject(SetObject.Request req) {
		
		SetObject.Response res = new SetObject.Response();
		res.success = false;
		
		if (req.apiKey != null && req.apiKey.length() > 0) {
			REInterface con = getREConnection(req.apiKey);
			OWLOntology ont = OWLIO.loadOntologyFromString(req.object);
			ArrayList<String> filenames = new ArrayList<String>();
			ArrayList<byte[]> dataArray = new ArrayList<byte[]>();
			for (ros.pkg.re_msgs.msg.File file : req.files) {
				filenames.add(file.name);
				dataArray.add(file.data);
			}
			res.success = con.submitObject(ont, req.cls, req.id, req.description, dataArray, filenames);	
		}
		
		return res;
		
	}

	public static DelObject.Response delObject(DelObject.Request req) {

		DelObject.Response res = new DelObject.Response();
		res.success = false;

		if (req.apiKey != null && req.apiKey.length() > 0) {

			REInterface con = getREConnection(req.apiKey);
			res.success = con.deleteObject(req.objectUID);

		}

		return res;

	}

	public static UpdateObject.Response updateObject(UpdateObject.Request req) {

		UpdateObject.Response res = new UpdateObject.Response();
		res.success = false;

		if (req.apiKey != null && req.apiKey.length() > 0) {

			REInterface con = getREConnection(req.apiKey);			
			OWLOntology ont = OWLIO.loadOntologyFromString(req.object);
			if (ont != null) {
				res.success = con.updateObject(req.uid, ont, req.description);	
			} else {
				res.success = false;
			}

		}

		return res;

	}

	public static SearchObjects.Response searchObjects(SearchObjects.Request req) {

		SearchObjects.Response res = new SearchObjects.Response();
		res.success = false;

		REInterface con = getREConnection(guestInterfaceKey);
		
		ArrayList<String> uids = new ArrayList<String>();
		ArrayList<ArrayList<String>> filenames = new ArrayList<ArrayList<String>>();
		ArrayList<ArrayList<String>> urls = new ArrayList<ArrayList<String>>();

		String[] onts = con.searchObjects(req.searchID, uids, filenames, urls);
		
		if (onts != null) {
			for (String ont : onts) {
				StringArray saNames = new StringArray();
				saNames.list = filenames.remove(0);
				res.filenames.add(saNames);
				
				StringArray saURLs = new StringArray();
				saURLs.list = urls.remove(0);
				res.fileURLs.add(saURLs);
				
				res.objects.add(ont);
			}
			res.uids = uids;
			res.success = true;
		} else {
			res.success = false;
		}

		return res;

	}
	
	public static QueryObjects.Response queryObjects(QueryObjects.Request req) {

		QueryObjects.Response res = new QueryObjects.Response();
		res.result = "";

		REInterface con = getREConnection(guestInterfaceKey);
		String result = con.queryObjectDB(req.query);
		if (result != null) {
			res.result = result;
		}

		return res;

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/get_object ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class GetObjectCallback implements ServiceServer.Callback<GetObject.Request, GetObject.Response> {

		@Override
		public GetObject.Response call(GetObject.Request req) {

			GetObject.Response res = getObject(req);
			if (res.success) {
				ros.logInfo("GetObject (UID: " + req.objectUID + "): Done");
			} else {
				ros.logInfo("GetObject (UID: " + req.objectUID + "): Failed");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/set_object ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SetObjectCallback implements ServiceServer.Callback<SetObject.Request, SetObject.Response> {

		@Override
		public SetObject.Response call(SetObject.Request req) {

			SetObject.Response res = setObject(req);
			
			if (req.apiKey != null && req.apiKey.length() > 0) {

				if (res.success) {
					ros.logInfo("SetObject (UID: " + req.cls+"."+req.id + "): Done");
				} else {
					ros.logInfo("SetObject (UID: " + req.cls+"."+req.id + "): Failed");
				}
				
			} else {
				ros.logInfo("SetObject (UID: " + req.cls+"."+req.id + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/del_object ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class DelObjectCallback implements ServiceServer.Callback<DelObject.Request, DelObject.Response> {

		@Override
		public DelObject.Response call(DelObject.Request req) {

			DelObject.Response res = delObject(req);
			
			if (req.apiKey != null && req.apiKey.length() > 0) {

				if (res.success) {
					ros.logInfo("DelObject (UID: " + req.objectUID + "): Done");
				} else {
					ros.logInfo("DelObject (UID: " + req.objectUID + "): Failed");
				}
				
			} else {
				ros.logInfo("DelObject (UID: " + req.objectUID + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/update_object ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class UpdateObjectCallback implements ServiceServer.Callback<UpdateObject.Request, UpdateObject.Response> {

		@Override
		public UpdateObject.Response call(UpdateObject.Request req) {

			UpdateObject.Response res = updateObject(req);
			
			if (req.apiKey != null && req.apiKey.length() > 0) {

				if (res.success) {
					ros.logInfo("UpdateObject (UID: " + req.uid + "): Done");
				} else {
					ros.logInfo("UpdateObject (UID: " + req.uid + "): Failed");
				}
				
			} else {
				ros.logInfo("UpdateObject (UID: " + req.uid + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/search_objects ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SearchObjectsCallback implements ServiceServer.Callback<SearchObjects.Request, SearchObjects.Response> {

		@Override
		public SearchObjects.Response call(SearchObjects.Request req) {

			SearchObjects.Response res = searchObjects(req);
			if (res.success) {
				ros.logInfo("SearchObjects (UID: " + req.searchID + "): Done");
			} else {
				ros.logInfo("SearchObjects (UID: " + req.searchID + "): Failed");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/query_objects ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class QueryObjectsCallback implements ServiceServer.Callback<QueryObjects.Request, QueryObjects.Response> {

		@Override
		public QueryObjects.Response call(QueryObjects.Request req) {

			QueryObjects.Response res = queryObjects(req);
			if (res.result != null && res.result.length() > 0) {
				ros.logInfo("QueryObjects (query:\n" + req.query + "): Done");
			} else {
				ros.logInfo("QueryObjects (query:\n" + req.query + "): Failed");
			}

			return res;

		}

	}

}
