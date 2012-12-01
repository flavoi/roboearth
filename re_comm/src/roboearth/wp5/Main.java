/* \file Main.java
 * \brief Main class for the re_comm package
 *
 * The main class for the re_comm package.
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
package roboearth.wp5;

import roboearth.wp5.module.ActionRecipeHandler;
import roboearth.wp5.module.BinaryFileHandler;
import roboearth.wp5.module.EnvironmentHandler;
import roboearth.wp5.module.MapExtractorHandler;
import roboearth.wp5.module.ObjectModelHandler;
import ros.NodeHandle;
import ros.Ros;

/**
 * 
 * The Main class consists of a main method in order to start up the ROS node.
 * 
 * @author Alexander Perzylo, perzylo@cs.tum.edu
 *
 */
public class Main {

	/**
	 * The main method initializes rosjava and creates a ROS node providing
	 * functionalities based on the loaded modules.  
	 */
	public static void main(String[] args) {

		boolean showUsage = false;
		boolean debug = false;

		for (String s : args) {
			if (s.startsWith("__")) { // ignore arguments added by roslaunch
				continue;
			}
			
			if (s.equalsIgnoreCase("--debug")) {
				debug = true;
			} else {
				showUsage = true;
				break;
			}
		}
		
		if (showUsage) {
			
			System.out.println("\nUsage:\n" +
					"re_comm accepts an optional argument, which enables " +
					"extensive logging to screen: --debug\n" +
					"\nExamples:\n" +
					"rosrun re_comm run\n" +
					"rosrun re_comm run --debug\n");
			return;
			
		} else if (debug) {
			
			// Setting log level for HttpClient
			java.util.logging.Logger.getLogger("org.apache.http.wire").setLevel(java.util.logging.Level.FINEST);
			java.util.logging.Logger.getLogger("org.apache.http.headers").setLevel(java.util.logging.Level.FINEST);
			System.setProperty("org.apache.commons.logging.Log", "org.apache.commons.logging.impl.SimpleLog");
			System.setProperty("org.apache.commons.logging.simplelog.showdatetime", "true");
			System.setProperty("org.apache.commons.logging.simplelog.log.httpclient.wire", "debug");
			System.setProperty("org.apache.commons.logging.simplelog.log.org.apache.http", "debug");
			System.setProperty("org.apache.commons.logging.simplelog.log.org.apache.http.headers", "debug");

			System.out.println("\nINFO: Enabled extensive logging to screen.");
			
		}
		
		// Initialize rosjava 
		Ros ros = Ros.getInstance();
		ros.init("re_comm_handler");

		// Create a NodeHandle
		NodeHandle n = ros.createNodeHandle();

		// Load modules providing specific functionalities 
		try {

			new ObjectModelHandler(ros, n);
			new BinaryFileHandler(ros, n);
			new ActionRecipeHandler(ros, n);
			new EnvironmentHandler(ros, n);
			new MapExtractorHandler(ros, n);

		} catch (Exception e) {

			if (ros != null) {
				ros.logFatal("Fatal error occurred. Shutting down!");	
			} else {
				System.out.println("Fatal error occurred. Shutting down!");
			}
			
			if (n != null) {
				n.shutdown();	
			}
			
			e.printStackTrace();
			return;
			
		}

		ros.spin();

	}
	
}
