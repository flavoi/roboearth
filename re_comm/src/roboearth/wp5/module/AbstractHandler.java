/* \file AbstractHandler.java
 * \brief Abstract handler
 *
 * The abstract handler provides the management of API keys/Connection objects
 * for handler implementations.
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

import java.util.TreeMap;

import roboearth.wp5.conn.REConnectionHadoop;
import roboearth.wp5.conn.REInterface;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;

public abstract class AbstractHandler {

	/**
	 * Reference to rosjava
	 */
	protected Ros ros;

	/**
	 * Node handle used to advertise the ROS services
	 */
	protected NodeHandle n;

	/**
	 * Key used for retrieving the guest interface to the RoboEarth DB (read access only)
	 */
	public final static String guestInterfaceKey = "re_guest";

	/**
	 * Mapping from API keys to personalized interfaces to the RoboEarth DB
	 */
	private static TreeMap<String, REInterface> conns = new TreeMap<String, REInterface>();
	static{
		conns.put(guestInterfaceKey, new REConnectionHadoop(null));
	}

	/**
	 * Constructor. Advertises the provided ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public AbstractHandler(Ros ros, NodeHandle n) throws RosException {

		this.ros = ros;
		this.n = n;

	}

	/**
	 * Searches for a RoboEarth connection object for a given API key and returns it. 
	 * If none could be found a new connection object gets created.  
	 * 
	 * @param apiKey the user's API key
	 * @return A REInterface object that uses the given API key for authentification
	 */
	public static REInterface getREConnection(String apiKey) {

		REInterface con = conns.get(apiKey);
		if (con == null) {
			con = new REConnectionHadoop(apiKey);
			conns.put(apiKey, con);
		}
		return con;

	}

}
