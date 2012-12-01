/* \file BinaryFileHandler.java
 * \brief Binary file handler
 *
 * The binary file handler provides the callbacks for ROS services related to 
 * binary files.
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

import roboearth.wp5.conn.REInterface;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.re_srvs.srv.DelEnvironmentBinaryFile;
import ros.pkg.re_srvs.srv.DelObjectBinaryFile;
import ros.pkg.re_srvs.srv.GetEnvironmentBinaryFile;
import ros.pkg.re_srvs.srv.GetObjectBinaryFile;
import ros.pkg.re_srvs.srv.SetEnvironmentBinaryFile;
import ros.pkg.re_srvs.srv.SetObjectBinaryFile;
import ros.pkg.re_srvs.srv.UpdateEnvironmentBinaryFile;
import ros.pkg.re_srvs.srv.UpdateObjectBinaryFile;

public class BinaryFileHandler extends AbstractHandler {

	public BinaryFileHandler(Ros ros, NodeHandle n) throws RosException {
		
		super(ros, n);

		this.n.advertiseService("/re_comm/get_object_binary_file", new GetObjectBinaryFile(), new GetObjectBinaryFileCallback());
		this.n.advertiseService("/re_comm/set_object_binary_file", new SetObjectBinaryFile(), new SetObjectBinaryFileCallback());
		this.n.advertiseService("/re_comm/del_object_binary_file", new DelObjectBinaryFile(), new DelObjectBinaryFileCallback());
		this.n.advertiseService("/re_comm/update_object_binary_file", new UpdateObjectBinaryFile(), new UpdateObjectBinaryFileCallback());

		this.n.advertiseService("/re_comm/get_environment_binary_file", new GetEnvironmentBinaryFile(), new GetEnvironmentBinaryFileCallback());
		this.n.advertiseService("/re_comm/set_environment_binary_file", new SetEnvironmentBinaryFile(), new SetEnvironmentBinaryFileCallback());
		this.n.advertiseService("/re_comm/del_environment_binary_file", new DelEnvironmentBinaryFile(), new DelEnvironmentBinaryFileCallback());
		this.n.advertiseService("/re_comm/update_environment_binary_file", new UpdateEnvironmentBinaryFile(), new UpdateEnvironmentBinaryFileCallback());
		
		ros.logInfo("Module 'BinaryFileHandler' loaded.");
		
	}

	/**
	 * 
	 * The callback class for the /re_comm/get_object_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class GetObjectBinaryFileCallback implements ServiceServer.Callback<GetObjectBinaryFile.Request, GetObjectBinaryFile.Response> {

		@Override
		public GetObjectBinaryFile.Response call(GetObjectBinaryFile.Request req) {

			GetObjectBinaryFile.Response res = new GetObjectBinaryFile.Response();

			REInterface con = getREConnection(guestInterfaceKey);
			byte[] data = con.requestObjectBinaryFile(req.objectUID, req.filename);
			
			if (data != null) {
				res.file.data = data;
				res.file.name = req.filename;
				res.success = true;
				ros.logInfo("GetBinaryFile (UID: "+req.objectUID+", " +
						"file: "+req.filename+"): Done");
			} else {
				res.success = false;
				ros.logInfo("GetBinaryFile (UID: "+req.objectUID+", " +
						"file: "+req.filename+"): Failed");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/set_object_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SetObjectBinaryFileCallback implements ServiceServer.Callback<SetObjectBinaryFile.Request, SetObjectBinaryFile.Response> {

		@Override
		public SetObjectBinaryFile.Response call(SetObjectBinaryFile.Request req) {

			SetObjectBinaryFile.Response res = new SetObjectBinaryFile.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.submitObjectBinaryFile(req.objectUID, req.file.data, req.file.name);
				if (res.success) {
					ros.logInfo("SetBinaryFile (UID: "+req.objectUID+", " +
							"file: "+req.file.name+"): Done");
				} else {
					ros.logInfo("SetBinaryFile (UID: "+req.objectUID+", " +
							"file: "+req.file.name+"): Failed");
				}

			} else {
				ros.logError("SetBinaryFile (UID: "+req.objectUID+", " +
						"file: "+req.file.name+"): API key is missing!");
			}

			return res;

		}

	}
	
	
	/**
	 * 
	 * The callback class for the /re_comm/del_object_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class DelObjectBinaryFileCallback implements ServiceServer.Callback<DelObjectBinaryFile.Request, DelObjectBinaryFile.Response> {

		@Override
		public DelObjectBinaryFile.Response call(DelObjectBinaryFile.Request req) {

			DelObjectBinaryFile.Response res = new DelObjectBinaryFile.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.deleteObjectBinaryFile(req.objectUID, req.filename);
				if (res.success) {
					ros.logInfo("DelBinaryFile (UID: "+req.objectUID+", " +
							"File: '"+req.filename+"'): Done");
				} else {
					ros.logInfo("DelBinaryFile (UID: "+req.objectUID+", " +
							"File: '"+req.filename+"'): Failed");
				}

			} else {
				ros.logError("DelBinaryFile (UID: " + req.objectUID + ", " +
						"File: '"+req.filename+"'): API key is missing!");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/update_object_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class UpdateObjectBinaryFileCallback implements ServiceServer.Callback<UpdateObjectBinaryFile.Request, UpdateObjectBinaryFile.Response> {

		@Override
		public UpdateObjectBinaryFile.Response call(UpdateObjectBinaryFile.Request req) {

			UpdateObjectBinaryFile.Response res = new UpdateObjectBinaryFile.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.updateObjectBinaryFile(req.objectUID, req.file.data, req.file.name);
				if (res.success) {
					ros.logInfo("UpdateBinaryFile (UID: " + req.objectUID + ", " +
						"File: '"+req.file.name+"'): Done");
				} else {
					ros.logInfo("UpdateBinaryFile (UID: " + req.objectUID + ", " +
						"File: '"+req.file.name+"'): Failed");
				}

			} else {
				ros.logError("UpdateBinaryFile (UID: " + req.objectUID + ", " +
						"File: '"+req.file.name+"'): API key is missing!");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/get_environment_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class GetEnvironmentBinaryFileCallback implements ServiceServer.Callback<GetEnvironmentBinaryFile.Request, GetEnvironmentBinaryFile.Response> {

		@Override
		public GetEnvironmentBinaryFile.Response call(GetEnvironmentBinaryFile.Request req) {

			GetEnvironmentBinaryFile.Response res = new GetEnvironmentBinaryFile.Response();

			REInterface con = getREConnection(guestInterfaceKey);
			byte[] data = con.requestEnvironmentBinaryFile(req.envUID, req.filename);
			
			if (data != null) {
				res.file.data = data;
				res.file.name = req.filename;
				res.success = true;
				ros.logInfo("GetBinaryFile (UID: "+req.envUID+", " +
						"file: "+req.filename+"): Done");
			} else {
				res.success = false;
				ros.logInfo("GetBinaryFile (UID: "+req.envUID+", " +
						"file: "+req.filename+"): Failed");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/set_environment_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SetEnvironmentBinaryFileCallback implements ServiceServer.Callback<SetEnvironmentBinaryFile.Request, SetEnvironmentBinaryFile.Response> {

		@Override
		public SetEnvironmentBinaryFile.Response call(SetEnvironmentBinaryFile.Request req) {

			SetEnvironmentBinaryFile.Response res = new SetEnvironmentBinaryFile.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.submitEnvironmentBinaryFile(req.envUID, req.file.data, req.file.name);
				if (res.success) {
					ros.logInfo("SetBinaryFile (UID: "+req.envUID+", " +
							"file: "+req.file.name+"): Done");
				} else {
					ros.logInfo("SetBinaryFile (UID: "+req.envUID+", " +
							"file: "+req.file.name+"): Failed");
				}

			} else {
				ros.logError("SetBinaryFile (UID: "+req.envUID+", " +
						"file: "+req.file.name+"): API key is missing!");
			}

			return res;

		}

	}
	
	
	/**
	 * 
	 * The callback class for the /re_comm/del_environment_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class DelEnvironmentBinaryFileCallback implements ServiceServer.Callback<DelEnvironmentBinaryFile.Request, DelEnvironmentBinaryFile.Response> {

		@Override
		public DelEnvironmentBinaryFile.Response call(DelEnvironmentBinaryFile.Request req) {

			DelEnvironmentBinaryFile.Response res = new DelEnvironmentBinaryFile.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.deleteEnvironmentBinaryFile(req.envUID, req.filename);
				if (res.success) {
					ros.logInfo("DelBinaryFile (UID: "+req.envUID+", " +
							"File: '"+req.filename+"'): Done");
				} else {
					ros.logInfo("DelBinaryFile (UID: "+req.envUID+", " +
							"File: '"+req.filename+"'): Failed");
				}

			} else {
				ros.logError("DelBinaryFile (UID: " + req.envUID + ", " +
						"File: '"+req.filename+"'): API key is missing!");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/update_environment_binary_file ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class UpdateEnvironmentBinaryFileCallback implements ServiceServer.Callback<UpdateEnvironmentBinaryFile.Request, UpdateEnvironmentBinaryFile.Response> {

		@Override
		public UpdateEnvironmentBinaryFile.Response call(UpdateEnvironmentBinaryFile.Request req) {

			UpdateEnvironmentBinaryFile.Response res = new UpdateEnvironmentBinaryFile.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.updateEnvironmentBinaryFile(req.envUID, req.file.data, req.file.name);
				if (res.success) {
					ros.logInfo("UpdateBinaryFile (UID: " + req.envUID + ", " +
						"File: '"+req.file.name+"'): Done");
				} else {
					ros.logInfo("UpdateBinaryFile (UID: " + req.envUID + ", " +
						"File: '"+req.file.name+"'): Failed");
				}

			} else {
				ros.logError("UpdateBinaryFile (UID: " + req.envUID + ", " +
						"File: '"+req.file.name+"'): API key is missing!");
			}

			return res;

		}

	}
	
}
