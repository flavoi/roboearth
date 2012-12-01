/*
 * Copyright (C) 2010 by Moritz Tenorth
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.roboearth.re_ontology;


import ros.*;
import ros.communication.Time;
import ros.pkg.re_msgs.msg.*;

/**
 * Dummy node that subscribes to the object poses published by Gazebo
 * and publishes them in the same format as the RoboEarth re_vision component.
 * 
 * To be obsolete once re_vision works properly.
 * 
 * @author tenorth
 *
 */
public class ReVisionDummyPublisher {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;

// 	Subscriber.QueueingCallback<ros.pkg.gazebo.msg.ModelStates> callback;
// 
// 	Thread listenToObjDetections;
// 	Thread publishReVisionMessages;
	
	
	
// 	/**
// 	 * Constructor: initializes the ROS environment
// 	 * 
// 	 * @param node_name A unique node name
// 	 */
// 	public ReVisionDummyPublisher(String node_name) {
// 		
// 		initRos(node_name);
// 		
// 		callback = new Subscriber.QueueingCallback<ros.pkg.gazebo.msg.ModelStates>();
// 		startObjDetectionsListener();
// 	}
// 
// 
// 	public void startObjDetectionsListener() {
// 
// 		// create threads for listening object detections
// 		
//         listenToObjDetections = new Thread( new ListenerThread() );
//         listenToObjDetections.start();
// 
//         publishReVisionMessages = new Thread( new PublishReVisionMsgsThread() );
//         publishReVisionMessages.start();
// 		
// 	}
	
	

// 	/**
// 	 * Initialize the ROS environment if it has not yet been initialized
// 	 * 
// 	 * @param node_name A unique node name
// 	 */
// 	protected static void initRos(String node_name) {
// 
// 		ros = Ros.getInstance();
// 
// 		if(!Ros.getInstance().isInitialized()) {
// 			ros.init(node_name);
// 		}
// 		n = ros.createNodeHandle();
// 
// 	}

	
// 	/**
// 	 * Listen the object poses published by Gazebo and buffer them
// 	 * 
// 	 * @author tenorth
// 	 *
// 	 */
// 	public class ListenerThread implements Runnable {
//     	
// 		@Override public void run() {
// 
// 			try {
// 
// 				Subscriber<ros.pkg.gazebo.msg.ModelStates> sub = 
// 					n.subscribe("/gazebo/model_states", new ros.pkg.gazebo.msg.ModelStates(), callback, 10);
// 
// 				n.spin();
// 				sub.shutdown();
// 
// 			} catch(RosException e) {
// 				e.printStackTrace();
// 			}
// 		}
// 	}
	

// 	/**
// 	 * Publish the re_vision messages
// 	 * 
// 	 * @author tenorth
// 	 *
// 	 */
// 	public class PublishReVisionMsgsThread implements Runnable {
// 
// 		@Override public void run() {
// 
// 			try {
// 
// 				ros.pkg.gazebo.msg.ModelStates res;
// 				
// 				Publisher<ros.pkg.re_msgs.msg.SeenObjectArray> pub =
// 					n.advertise("/vslam/seen_objects", new ros.pkg.re_msgs.msg.SeenObjectArray(), 100);
// 
// 				
// 				int i_msg=0;
// 				while (n.isValid()) {
// 
// 					res = callback.pop();
// 
// 					if( ((i_msg++)%1000) == 0) {
// 
// 	        			// iterate over detected objects
// 						ros.pkg.re_msgs.msg.SeenObjectArray msg = new ros.pkg.re_msgs.msg.SeenObjectArray();
// 						int i_obj=0;
// 						
// 	        			for(int i_res=0; i_res<res.name.size();i_res++) {
// 
// 	        				if( (res.name.get(i_res).equals("bed")) ||
// 		        				(res.name.get(i_res).equals("cabinet")) ||
// 		        				(res.name.get(i_res).equals("bottle1")) ||
// 		        				(res.name.get(i_res).equals("chair")) ) {
// 	        					
// 	        						SeenObject new_obj = new SeenObject();
// 	        						new_obj.name = res.name.get(i_res);
// 	        						new_obj.pose = res.pose.get(i_res).clone();
// 	        						new_obj.stamp = new Time();
// 	        						msg.object.add(new_obj);
// 	        						
// 	        						i_obj++;
// 
// 	        				}
// 
// 	        			}
// 
// 						pub.publish(msg);
// 	        			System.out.println("Publishing");
// 					}
//     				n.spinOnce();
//     				
// 				}
// 				pub.shutdown();
// 
// 			} catch (Exception e) {
// 				e.printStackTrace();
// 			}
// 		}
// 	}

	
// 	public static void main(String[] args) {
// 		
// 		new ReVisionDummyPublisher("re_vision_dummy");
// 	}
}
