/* \file REInterface.java
 * \brief Interface definition for the RoboEarth DB
 *
 * The interface definition for interacting with the RoboEarth DB. It provides 
 * means to download/upload/update/delete/query action recipes, object models 
 * and environments.
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
package roboearth.wp5.conn;

import java.io.File;
import java.util.ArrayList;

import org.semanticweb.owlapi.model.OWLOntology;

public interface REInterface {

	/**
	 * Requests an action recipe.
	 * 
	 * @param uid the action recipe's unique identifier
	 * @return String object containing OWL description of an action recipe
	 */
	public String requestActionRecipe(String uid);

	/**
	 * Requests a description of an environment. The names and URLs of binary 
	 * files related to the given uid will be put into the given lists. Any 
	 * element that might be in the lists before the method call will 
	 * be removed first. Both list parameters may be null. In this case  
	 * only the OWL encoded description is requested.
	 * 
	 * @param uid the environment's identifier
	 * @param outFilenames this list will be used to return the names
	 * of associated files
	 * @param outFileURLs this list will be used to return the URLs
	 * of associated files
	 * @return String object containing OWL description of environmental data
	 */
	public String requestEnvironment(String uid, ArrayList<String> outFilenames, 
			ArrayList<String> outFileURLs);

	/**
	 * Requests information about an object. The names and URLs of binary 
	 * files related to the given uid will be put into the given lists. Any 
	 * element that might be in the lists before the method call will 
	 * be removed first. Both list parameters may be null. In this case  
	 * only the OWL encoded description is requested.
	 * 
	 * @param uid the object's unique identifier 
	 * @param outFilenames this list will be used to return the names
	 * of associated files
	 * @param outFileURLs this list will be used to return the URLs
	 * of associated files
	 * @return String object containing OWL description of the object
	 */
	public String requestObject(String uid, ArrayList<String> outFilenames, 
			ArrayList<String> outFileURLs);
	
	/**
	 * Requests a robot description.
	 * 
	 * @param uid the robot's unique identifier
	 * @return String object containing SRDL description of a robot
	 */
	public String requestRobot(String uid);
	
	/**
	 * Requests an object related binary file.
	 * 
	 * @param objectUID the object's ID the file is related to 
	 * @param filename the unique filename (among all files related to the 
	 * specified object)
	 * @param targetPath the path where the file shall be stored
	 * @return the requested file or null, if the file couldn't be found
	 */
	public File requestObjectBinaryFile(String objectUID, String filename, String targetPath);

	/**
	 * Requests an object related binary file.
	 * 
	 * @param objectUID the object's ID the file is related to
	 * @param filename the unique filename (among all files related to the 
	 * specified object)
	 * @return the content of the requested file in a byte array
	 */
	public byte[] requestObjectBinaryFile(String objectUID, String filename);
	
	/**
	 * Requests an environment related binary file.
	 * 
	 * @param envUID the environment's ID the file is related to 
	 * @param filename the unique filename (among all files related to the 
	 * specified environment)
	 * @param targetPath the path where the file shall be stored
	 * @return the requested file or null, if the file couldn't be found
	 */
	public File requestEnvironmentBinaryFile(String envUID, String filename, String targetPath);
	
	/**
	 * Requests an environment related binary file.
	 * 
	 * @param envUID the environment's ID the file is related to
	 * @param filename the unique filename (among all files related to the 
	 * specified environment)
	 * @return the content of the requested file in a byte array
	 */
	public byte[] requestEnvironmentBinaryFile(String envUID, String filename);
	
	/**
	 * Submits an action recipe to the RoboEarth DB.
	 * 
	 * @param actionRecipe OWLOntology object containing the action recipe
	 * @param cls the class the action recipe belongs to
	 * @param id the action recipe's identifier
	 * @param description a natural language description of the recipe's 
	 * functionality
	 * @return <tt>true</tt> - if submission was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean submitActionRecipe(OWLOntology actionRecipe,
			String cls, String id, String description);

	/**
	 * Submits a description of an environment to the RoboEarth DB.
	 * 
	 * @param env OWLOntology object containing the environment's description
	 * @param cls the class the environment belongs to
	 * @param id the environment's identifier
	 * @param description a natural language description of the environment
	 * @return <tt>true</tt> - if submission was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean submitEnvironment(OWLOntology env, String cls, String id,
			String description);

	public boolean submitEnvironment(OWLOntology env, String cls, String id, 
			String description, ArrayList<File> binaryFiles);
	
	public boolean submitEnvironment(OWLOntology env, String cls, String id, 
			String description, ArrayList<byte[]> binaryData, 
			ArrayList<String> filenames);
	
	/**
	 * Submits an object's description to the RoboEarth DB. 
	 * 
	 * @param object_owl OWLOntology object containing the object's 
	 * description
	 * @param cls the class the object belongs to
	 * @param id the object's identifier
	 * @param description a natural language description of the object
	 * @return <tt>true</tt> - if submission was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean submitObject(OWLOntology objectOwl, String cls, String id, 
			String description);
	
	/**
	 * Submits an object's description to the RoboEarth DB. The given
	 * binary files, related to the object description, will be stored 
	 * in the RoboEarthDB as well.  
	 * 
	 * @param objectOwl OWLOntology object containing the object's 
	 * description
	 * @param cls the class the object belongs to
	 * @param id the object's identifier
	 * @param description a natural language description of the object
	 * @param binaryFiles object related binary files
	 * @return <tt>true</tt> - if submission was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean submitObject(OWLOntology objectOwl, String cls, String id, 
			String description, ArrayList<File> binaryFiles);
	
	/**
	 * Submits an object's description to the RoboEarth DB. The given
	 * binary data, related to the object description, will be stored 
	 * in the RoboEarthDB as well.
	 * 
	 * @param objectOwl OWLOntology object containing the object's 
	 * description
	 * @param cls the class the object belongs to
	 * @param id the object's identifier
	 * @param description a natural language description of the object
	 * @param binaryData the content of the files in byte arrays
	 * @param filenames the unique filenames (among all files related to the 
	 * specified object)
	 * @return <tt>true</tt> - if submission was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean submitObject(OWLOntology objectOwl, String cls, String id, 
			String description, ArrayList<byte[]> binaryData, 
			ArrayList<String> filenames);
	
	/**
	 * Submits a binary file (related to the resource specified by the given 
	 * object UID) to the RoboEarth DB. The filename must be unique among 
	 * all files related to the same object uid. 
	 * 
	 * @param uid the unique identifier of the resource the given binary 
	 * file is linked with
	 * @param file file to submit 
	 * @return <tt>true</tt> if binary file was submitted successfully, 
	 * <tt>false</tt> otherwise
	 */
	public boolean submitObjectBinaryFile(String uid, File file);

	/**
	 * Submits a binary file (related to the resource specified by the given 
	 * object UID) to the RoboEarth DB. The filename must be unique among 
	 * all files related to the same object uid.
	 * 
	 * @param uid the unique identifier of the resource the given binary 
	 * file is linked with
	 * @param data the content of the file in a byte array
	 * @param filename the unique filename (among all files related to the 
	 * specified object)
	 * @return <tt>true</tt> if binary file was submitted successfully, 
	 * <tt>false</tt> otherwise
	 */
	public boolean submitObjectBinaryFile(String uid, byte[] data, String filename);
	
	public boolean submitEnvironmentBinaryFile(String uid, File file);
	
	public boolean submitEnvironmentBinaryFile(String uid, byte[] data, String filename);
	
	/**
	 * Deletes an action recipe from the RoboEarth DB.
	 * 
	 * @param uid the action recipe's unique identifier
	 * @return <tt>true</tt> - if the recipe was successfully deleted<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean deleteActionRecipe(String uid);

	/**
	 * Deletes an environment from the RoboEarth DB.
	 * 
	 * @param uid the environment's unique identifier
	 * @return <tt>true</tt> - if the environment was successfully deleted<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean deleteEnvironment(String uid);

	/**
	 * Deletes an object description from the RoboEarth DB. All linked 
	 * binary files get deleted, too.
	 * 
	 * @param uid the object's unique identifier
	 * @return <tt>true</tt> - if the object description was successfully
	 * deleted<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean deleteObject(String uid);

	/**
	 * Deletes a binary file which is related to the specified object
	 * description.
	 * 
	 * @param uid the object's unique identifier
	 * @param filename the unique filename (among all files related to the 
	 * specified object)
	 * @return <tt>true</tt> - if the file was successfully deleted<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean deleteObjectBinaryFile(String uid, String filename);
	
	/**
	 * Deletes a binary file which is related to the specified environment
	 * description.
	 * 
	 * @param uid the environment's unique identifier
	 * @param filename the unique filename (among all files related to the 
	 * specified environment)
	 * @return <tt>true</tt> - if the file was successfully deleted<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean deleteEnvironmentBinaryFile(String uid, String filename);
	
	/**
	 * Searches for action recipes whose uid starts with the given search
	 * string. 
	 * 
	 * @param searchID any prefix of an uid 
	 * @return an array of String objects containing OWL descriptions of the action recipes 
	 * found 
	 */
	public String[] searchActionRecipes(String searchID, ArrayList<String> outUIDs);
	
	/**
	 * Searches for environments whose uid starts with the given search
	 * string.
	 * 
	 * @param searchID any prefix of an uid
	 * @param outFilenames this list of lists will be used to return for every uid found
	 * the names of associated files. The first dimension's index corresponds to 
	 * the index of the related owl description in the array that is returned by 
	 * this method. 
	 * @param outFileURLs this list of lists will be used to return for every uid found
	 * the URLs of associated files. The first dimension's index corresponds to 
	 * the index of the related owl description in the array that is returned by 
	 * this method.
	 * @return an array of String objects containing OWL descriptions of the environments 
	 * found
	 */
	public String[] searchEnvironments(String searchID, ArrayList<String> outUIDs, 
			ArrayList<ArrayList<String>> outFilenames, 
			ArrayList<ArrayList<String>> outFileURLs);
	
	/**
	 * Searches for object descriptions whose uid starts with the given search
	 * string.
	 * 
	 * @param searchID any prefix of an uid
	 * @param outUIDs this list will be used to return the UIDs of all found objects
	 * @param outFilenames this list of lists will be used to return for every uid found
	 * the names of associated files. The first dimension's index corresponds to 
	 * the index of the related owl description in the array that is returned by 
	 * this method. 
	 * @param outFileURLs this list of lists will be used to return for every uid found
	 * the URLs of associated files. The first dimension's index corresponds to 
	 * the index of the related owl description in the array that is returned by 
	 * this method.
	 * @return an array of String objects containing OWL descriptions of the objects 
	 * found
	 */
	public String[] searchObjects(String searchID, ArrayList<String> outUIDs, 
			ArrayList<ArrayList<String>> outFilenames, 
			ArrayList<ArrayList<String>> outFileURLs);
	
	/**
	 * Sends the given SeRQL query to the RoboEarth DB and retrieves the 
	 * result of the server-side reasoning on action recipes.
	 * 
	 * @param seRQLquery a SeRQL query
	 * @return result as a String
	 */
	public String queryActionRecipeDB(String seRQLquery);

	/**
	 * Sends the given SeRQL query to the RoboEarth DB and retrieves the 
	 * result of the server-side reasoning on environments.
	 * 
	 * @param seRQLquery a SeRQL query
	 * @return result as a String
	 */
	public String queryEnvironmentDB(String seRQLquery);
	
	/**
	 * Sends the given SeRQL query to the RoboEarth DB and retrieves the 
	 * result of the server-side reasoning on objects.
	 * 
	 * @param seRQLquery a SeRQL query
	 * @return result as a String
	 */
	public String queryObjectDB(String seRQLquery);

	/**
	 * Updates the action recipe specified by the parameter uid with 
	 * the given OWL and natural language descriptions.
	 * 
	 * @param uid the action recipe's unique identifier
	 * @param actionRecipe OWLOntology object containing the action recipe
	 * @param description a natural language description of the recipe's 
	 * functionality
	 * @return <tt>true</tt> - if the recipe was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean updateActionRecipe(String uid, OWLOntology actionRecipe, String description);

	/**
	 * Updates the environment specified by the parameter uid with 
	 * the given OWL and natural language descriptions.
	 *
	 * @param uid the environment's unique identifier
	 * @param env OWLOntology object containing the environment's description
	 * @param description a natural language description of the environment
	 * @return <tt>true</tt> - if the environment was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean updateEnvironment(String uid, OWLOntology env, String description);

	/**
	 * Updates the object specified by the parameter uid with 
	 * the given OWL and natural language descriptions.
	 *
	 * @param uid the object's unique identifier
	 * @param objectOwl OWLOntology object containing the object's 
	 * description
	 * @param description a natural language description of the object
	 * @return <tt>true</tt> - if the object was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean updateObject(String uid, OWLOntology objectOwl, String description);

	public boolean updateObject(String uid, String objectOwl, String description);
	
	/**
	 * Updates (replaces) the specified object related binary file with the 
	 * given file.
	 *
	 * @param uid the object's unique identifier
	 * @return <tt>true</tt> - if the file was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean updateObjectBinaryFile(String uid, File file);
	
	/**
	 * Updates (replaces) the specified object related binary file with the 
	 * content of the given byte array.
	 * 
	 * @param uid the object's unique identifier
	 * @param data the content of the file in a byte array
	 * @param filename the unique filename (among all files related to the 
	 * specified object)
	 * @return <tt>true</tt> - if the file was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean updateObjectBinaryFile(String uid, byte[] data, String filename);
	
	/**
	 * Updates (replaces) the specified environment related binary file with the 
	 * given file.
	 *
	 * @param uid the environment's unique identifier
	 * @return <tt>true</tt> - if the file was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean updateEnvironmentBinaryFile(String uid, File file);
	
	/**
	 * Updates (replaces) the specified environment related binary file with the 
	 * content of the given byte array.
	 * 
	 * @param uid the environment's unique identifier
	 * @param data the content of the file in a byte array
	 * @param filename the unique filename (among all files related to the 
	 * specified environment)
	 * @return <tt>true</tt> - if the file was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	public boolean updateEnvironmentBinaryFile(String uid, byte[] data, String filename);

	
	public ArrayList<byte[]> request2dMap(String envUid, OWLOntology srdl, String baseScannerLink, String simpleMapNameWithoutExt);
	public ArrayList<byte[]> requestProjected2dMap(String envUid, double minZ, double maxZ, String simpleMapNameWithoutExt);

}
