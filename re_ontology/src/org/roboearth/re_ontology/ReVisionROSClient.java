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

import java.util.HashMap;
import java.util.Hashtable;
import java.util.Vector;

import jpl.Query;
import ros.*;
import ros.pkg.re_msgs.msg.*;
import ros.pkg.geometry_msgs.msg.*;


public class ReVisionROSClient {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;

	Subscriber.QueueingCallback<ros.pkg.re_msgs.msg.SeenObjectArray> callback;

	Thread listenToObjDetections;
	Thread updateKnowRobObjDetections;



	/**
	 * Constructor: initializes the ROS environment
	 *
	 * @param node_name A unique node name
	 */
	public ReVisionROSClient(String node_name) {

		initRos(node_name);

		callback = new Subscriber.QueueingCallback<ros.pkg.re_msgs.msg.SeenObjectArray>();

	}



	public void startObjDetectionsListener(String re_vision_topic) {

		// create threads for listening to object detections

        listenToObjDetections = new Thread( new ListenerThread(re_vision_topic) );
        listenToObjDetections.start();

        updateKnowRobObjDetections = new Thread( new UpdateKnowrobThread() );
        updateKnowRobObjDetections.start();

	}



	/**
	 * Initialize the ROS environment if it has not yet been initialized
	 *
	 * @param node_name A unique node name
	 */
	protected static void initRos(String node_name) {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init(node_name);
		}
		n = ros.createNodeHandle();

	}


	/**
	 * Thread for listening to the object detections, puts the results into a
	 * QueuingCallback buffer
	 *
	 * @author tenorth
	 *
	 */
	public class ListenerThread implements Runnable {

    	String topic;

    	public ListenerThread() {
			//this("/vslam/SeenObjects");
    		this("/re_world_model/filtered_object_locations");
		}

    	public ListenerThread(String t) {
			topic=t;
		}

		@Override public void run() {

			try {

				Subscriber<ros.pkg.re_msgs.msg.SeenObjectArray> sub =
					n.subscribe(topic, new ros.pkg.re_msgs.msg.SeenObjectArray(), callback, 10);

				n.spin();
				sub.shutdown();

			} catch(RosException e) {
				e.printStackTrace();
			}
		}
	}


	/**
	 * Read perceptions from the QueueingCallback buffer and create the
	 * corresponding object representations in KnowRob.
	 *
	 * @author tenorth
	 *
	 */
	public class UpdateKnowrobThread implements Runnable {

		public UpdateKnowrobThread() {

		}

		@Override public void run() {

			try {

				ros.pkg.re_msgs.msg.SeenObjectArray res;
				HashMap<String, Vector<Object>> solutions;

				while (n.isValid()) {

					res = callback.pop();

        			// iterate over detected objects
        			for(SeenObject seen_obj : res.object) {

        				int obj_id = 1; // TODO: use a real object ID when provided by the perception
        				String obj_name = seen_obj.name;


        				// create VisualPerception instance
        				//System.err.println("comp_re_vision:re_create_perception_instance(Perception)");
        				solutions = executeQuery("comp_re_vision:re_create_perception_instance(Perception)");
        				if(solutions.get("Perception").size()>1) {throw new Exception("ERROR: More than one Perception instance created.");}
        	    		String perception = solutions.get("Perception").get(0).toString();

        	    		// set the pose
        	    		//System.err.println("comp_re_vision:re_set_perception_pose("+perception+", "+doubleArrayToPlList(objectPose(seen_obj))+")");
        				solutions = executeQuery("comp_re_vision:re_set_perception_pose("+perception+", "+doubleArrayToPlList(objectPose(seen_obj))+")");

        				// create object information
        	    		//System.err.println("comp_re_vision:re_create_object_instance(["+obj_name+"], "+obj_id+", Obj)");
        				solutions = executeQuery("comp_re_vision:re_create_object_instance(["+obj_name+"], "+obj_id+", Obj)");
        				if(solutions.get("Obj").size()>1) {throw new Exception("ERROR: More than one Object instance created:"+objectArrayToPlList(solutions.get("Obj").toArray()));}
        				String obj_inst = solutions.get("Obj").get(0).toString();

        				synchronized(jpl.Query.class) {
							// link VisualPerception instance to the object instance
							new Query("comp_re_vision:re_set_object_perception("+obj_inst+", "+perception+")").allSolutions();
        				}
        			}
    				n.spinOnce();
				}
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Convert a Java array into a Prolog list to be used in the string-based query interface
	 *
	 * @param a The array to be converted
	 * @return A Prolog list of the form ['a0', 'a1']
	 */
	protected static String objectArrayToPlList(Object[] a) {
		String res="[";
		for(int i=0;i<a.length;i++) {

			if(a[i].toString().startsWith("'"))
				res+=a[i].toString();
			else
				res+="'"+a[i].toString()+"'";

			if(i<a.length-1)
				res+=",";
		}
		return res+="]";
	}

	protected static String doubleArrayToPlList(double[] a) {
		String res="[";
		for(int i=0;i<a.length;i++) {
			res+="'"+a[i]+"'";
			if(i<a.length-1)
				res+=",";
		}
		return res+="]";
	}


	/**
	 * Read the pose of an object from the SeenObject (detection result)
	 * @param p A detected object as SeenObject
	 * @return Row-based 4x4 pose matrix representation
	 */
	public static double[] objectPose(SeenObject p) {

		return quaternionToMatrix(p.pose.position, p.pose.orientation);
	}

	/**
	 * Convert point/quaternion into a 4x4 pose matrix
	 *
	 * @param p Point (position)
	 * @param q Quaternion (orientation)
	 * @return 4x4 pose matrix, row-based
	 */
	protected static double[] quaternionToMatrix(Point p, Quaternion q) {

		double[] m = new double[16];

	    double xx = q.x * q.x;
	    double xy = q.x * q.y;
	    double xz = q.x * q.z;
	    double xw = q.x * q.w;

	    double yy = q.y * q.y;
	    double yz = q.y * q.z;
	    double yw = q.y * q.w;

	    double zz = q.z * q.z;
	    double zw = q.z * q.w;

	    m[0]  = 1 - 2 * ( yy + zz );
	    m[1]  =     2 * ( xy - zw );
	    m[2]  =     2 * ( xz + yw );
	    m[3]  = p.x;

	    m[4]  =     2 * ( xy + zw );
	    m[5]  = 1 - 2 * ( xx + zz );
	    m[6]  =     2 * ( yz - xw );
	    m[7]  = p.y;

	    m[8]  =     2 * ( xz - yw );
	    m[9]  =     2 * ( yz + xw );
	    m[10] = 1 - 2 * ( xx + yy );
	    m[11]=p.z;

	    m[12]=0;
	    m[13]=0;
	    m[14]=0;
	    m[15]=1;
	    return m;
	}


	/**
	 * Wrapper around the JPL Prolog interface
	 *
	 * @param query A query string in common SWI Prolog syntax
	 * @return A HashMap<VariableName, ResultsVector>
	 */
	public static HashMap<String, Vector<Object>> executeQuery(String query) {

		HashMap<String, Vector<Object>> result = new HashMap< String, Vector<Object> >();
		Hashtable[] solutions;

		synchronized(jpl.Query.class) {

    		Query q = new Query( "expand_goal(("+query+"),_9), call(_9)" );

    		if(!q.hasSolution())
    			return new HashMap<String, Vector<Object>>();


    		solutions = q.allSolutions();
    		for (Object key: solutions[0].keySet()) {
    			result.put(key.toString(), new Vector<Object>());
    		}

    		// Build the result
    		for (int i=0; i<solutions.length; i++) {
    			Hashtable solution = solutions[i];
    			for (Object key: solution.keySet()) {
    				String keyStr = key.toString();
    				if (!result.containsKey( keyStr )) {

    					// previously unknown column, add result vector
    					Vector<Object> resultVector = new Vector<Object>();
    					resultVector.add( i, solution.get( key ).toString() );
    					result.put(keyStr, resultVector);

    				}
    				// Put the solution into the correct vector
    				Vector<Object> resultVector = result.get( keyStr );
    				resultVector.add( i, solution.get( key ).toString() );
    			}
    		}
		}
		// Generate the final QueryResult and return
		return result;
	}


}
