/* \file REClients.java
 * \brief RoboEarth Client code for being called from Prolog
 * 
 * This file is part of the RoboEarth ROS re_comm package.
 * 
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2010 by 
 * <a href=" mailto:tenorth@cs.tum.edu">Moritz Tenorth</a>
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
 * \author Moritz Tenorth
 * \version 1.0
 * \date 2010
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 */
package roboearth.wp5;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.StringReader;
import java.net.URL;
import java.util.ArrayList;
import java.util.Set;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParserFactory;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.xml.sax.Attributes;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.DefaultHandler;

import roboearth.wp5.conn.REConnectionHadoop;
import roboearth.wp5.owl.OWLIO;
import roboearth.wp5.util.Util;
import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.communication.Time;
import ros.pkg.re_msgs.msg.SeenObject;
import ros.pkg.re_srvs.srv.DetectObjects;
import ros.pkg.re_srvs.srv.LoadVslamMap;
import ros.pkg.re_srvs.srv.RoboEarthExportCopModel;
import ros.pkg.re_srvs.srv.RoboEarthRetrieveCopModel;

import com.google.common.base.CaseFormat;
import com.google.common.base.Joiner;

import de.tum.in.fipm.kipm.gui.visualisation.applets.CommunicationVisApplet;


public class REClients {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;

	static final String API_KEY = "6e6574726f6d40b699e442ebdca5850e7cb7486679768aec3c70";

	static ArrayList<String> obj_urls = new ArrayList<String>();
	static ArrayList<String> act_urls = new ArrayList<String>();
	static ArrayList<String> env_urls = new ArrayList<String>();
	

	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

		ros = Ros.getInstance();

		if(!ros.isInitialized()) {
			ros.init("knowrob_re_client");
		}
		n = ros.createNodeHandle();

	}

	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download object information from the DB
	//

	
	/**
	 * Sends a 'semantic query' to the RoboEarth DB to search for models that 
	 * re:providesModelFor the desired object class
	 * 
	 * @param objclass The object class a model is searched for
	 * @return Array of URL strings pointing to the object models
	 * @throws IOException
	 * @throws ParserConfigurationException
	 * @throws SAXException
	 */
	public static String[] requestModelFor(String objclass) throws IOException, ParserConfigurationException, SAXException {
		
		String q = "SELECT source FROM CONTEXT source\n" +
				 "{S} roboearth:providesModelFor {"+objclass.replace("'", "")+"}\n" +
				 "USING NAMESPACE\n" +
				 "roboearth=<http://www.roboearth.org/kb/roboearth.owl#>,\n"+
				 "knowrob=<http://ias.cs.tum.edu/kb/knowrob.owl#>";
		
		CommunicationVisApplet.visualizeCommunication("Requesting model for object '"+objclass.replace("'", "")+"' from RoboEarth..." + q, "", "amigo.jpg", "roboearth.png");
//		System.err.println("\nRequesting model for: " + objclass.replace("'", ""));
		
		String res;
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		
		res = conn.queryObjectDB(q);
		
		res = res.replace("\\n", "").replace("\\t", "");
		res = res.substring(1, res.length()-1);
		
		REClients.obj_urls.clear();
		SAXParserFactory factory = SAXParserFactory.newInstance();
		factory.setValidating(false);
		REClients rec = new REClients();
		factory.newSAXParser().parse(new InputSource(new StringReader(res)), rec.new SparqlObjReader());
		
		return obj_urls.toArray(new String[0]);
	}
	
	
	
	public static String downloadModelFrom(String url) throws IOException, ParserConfigurationException, SAXException {

		ArrayList<String> outFilenames = new ArrayList<String>();
		ArrayList<String> outFileURLs = new ArrayList<String>();
		
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		
		String objDir = Util.tmpDir + Util.getFilenameFromURL(url); 
		String filename = objDir +".owl";
		
		FileWriter out = new FileWriter(filename);
		String obj = conn.requestObjectFromURL(url, outFilenames, outFileURLs);
		out.write(obj==null?"":obj);
		out.close();
		
		if (outFileURLs.size() > 0) {
			File dir = new File(objDir);
			if (dir.exists()) {
				Util.deleteFolderRec(dir, false);	
			} else {
				dir.mkdir();	
			}

			for(int i=0;i<outFileURLs.size();i++) {
				conn.requestBinaryFile(new URL(outFileURLs.get(i)), objDir);
			}
		}

//		System.err.print("Model downloaded\n\n\n");
		
		CommunicationVisApplet.visualizeCommunication("", "Received object model "+ Util.getFilenameFromURL(url), null, "roboearth.png");
		return filename;
	}
	
	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download action recipes from the DB
	//
	
	/**
	 * Sends a 'semantic query' to the RoboEarth DB to search for action recipes that 
	 * perform the task specified as a rdfs:label
	 * 
	 * @param command String command for which a recipe is searched
	 * @return Array of URL strings pointing to the recipe specifications
	 * @throws IOException
	 * @throws ParserConfigurationException
	 * @throws SAXException
	 */
	public static String[] requestActionRecipeFor(String command) throws IOException, ParserConfigurationException, SAXException {


		String q = "SELECT source FROM CONTEXT source\n" +
				 "{S} rdfs:label {\""+command.replace("'", "")+"\"^^xsd:string}\n" +
				 "USING NAMESPACE\n" +
				 "rdfs=<http://www.w3.org/2000/01/rdf-schema#>\n";
		
		CommunicationVisApplet.visualizeCommunication("Requesting action recipe for '"+command.replace("'", "")+"' from RoboEarth...\n"+q, "", "amigo.jpg", "roboearth.png");
		
		String res;
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		
		res = conn.queryActionRecipeDB("SELECT source FROM CONTEXT source\n" +
				 "{S} rdfs:label {\""+command.replace("'", "")+"\"^^xsd:string}\n" +
				 "USING NAMESPACE\n" +
				 "rdfs=<http://www.w3.org/2000/01/rdf-schema#>\n");
		
		res = res.replace("\\n", "").replace("\\t", "");
		res = res.substring(1, res.length()-1);
		//System.out.println(res);
		
		REClients.act_urls.clear();
		SAXParserFactory factory = SAXParserFactory.newInstance();
		factory.setValidating(false);
		REClients rec = new REClients();
		factory.newSAXParser().parse(new InputSource(new StringReader(res)), rec.new SparqlRecipeReader());
		
		return act_urls.toArray(new String[0]);
	}
	

	public static String downloadRecipeFrom(String url) throws IOException, ParserConfigurationException, SAXException {

		//System.err.println(url);
		
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		String filename = Util.tmpDir + Util.getFilenameFromURL(url)+".owl";
		
		FileWriter out = new FileWriter(filename);
		out.write(conn.requestActionRecipeFromURL(url));
		out.close();

		CommunicationVisApplet.visualizeCommunication(null, "Received recipe "+ Util.getFilenameFromURL(url), "amigo.jpg", "roboearth.png");

		return filename;
	}

	
	

	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download map information from the DB
	//


	/**
	 * Sends a 'semantic query' to the RoboEarth DB to search for an environment model
	 * 
	 * @param roomNumber Environment for which to search (currently: room number)
	 * @return Array of URL strings pointing to the recipe specifications
	 * @throws IOException
	 * @throws ParserConfigurationException
	 * @throws SAXException
	 */
	public static String[] requestEnvironmentMapFor(String[][] roomQuery) throws IOException, ParserConfigurationException, SAXException {
	
		String q = "SELECT source FROM CONTEXT source\n" +
				"{A} kr:describedInMap {Z} ,\n";
		
		ArrayList<String> constr = new ArrayList<String>();
		ArrayList<String> where  = new ArrayList<String>();
		
		char idx = 'A';
		for(String[] constraint : roomQuery) {
			
			if(idx!='A'){
				constr.add("{"+idx+"} kr:properPhysicalParts {"+(char)(idx-1)+"}");	
			}
			
			String var = constraint[0].split(":")[1]; 
			var = CaseFormat.LOWER_CAMEL.to(CaseFormat.UPPER_CAMEL, var);
			var = "V"+var;  // avoid problems with reserved words like 'label' 
			
			constr.add("{"+idx+"} " + constraint[0] + " {"+var+"}");
			where.add(var+" LIKE \""+constraint[1]+"\"");
			
			idx++;
		}
		
		q+= Joiner.on(" , \n").join(constr);
		q+="\nWHERE\n" + Joiner.on("\nAND ").join(where);
		
		q += "\nUSING NAMESPACE\n" +
			"re=<http://www.roboearth.org/kb/roboearth.owl#>,\n" + 
			"rdfs=<http://www.w3.org/2000/01/rdf-schema#>,\n" +
			"kr=<http://ias.cs.tum.edu/kb/knowrob.owl#> ";
		
		CommunicationVisApplet.visualizeCommunication("Requesting map from RoboEarth..." + q, "", "amigo.jpg", "roboearth.png");
		
		String res;
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);

		
		res = conn.queryEnvironmentDB(q);

		res = res.replace("\\n", "").replace("\\t", "");
		res = res.substring(1, res.length()-1);
				
		REClients.env_urls.clear();
		SAXParserFactory factory = SAXParserFactory.newInstance();
		factory.setValidating(false);
		REClients rec = new REClients();
		factory.newSAXParser().parse(new InputSource(new StringReader(res)), rec.new SparqlEnvReader());
		
		return env_urls.toArray(new String[0]);
	}


	public static String downloadEnvironmentMapFrom(String url, String robotUID, ArrayList<String> outFilenames) throws IOException, ParserConfigurationException, SAXException {

		ArrayList<String> outFileURLs = new ArrayList<String>();
		
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		
		String envDir = Util.tmpDir + Util.getFilenameFromURL(url); 
		String filename = envDir + ".owl";
		
		FileWriter out = new FileWriter(filename);
		String env = conn.requestEnvironmentFromURL(url);
		if (env != null) {
			out.write(env);
			out.close();			
		} else {
			System.err.println("Error: environment '" + url + "' couldn't be found.");
			return null;
		}
		
		File dir = new File(envDir);
		if (dir.exists()) {
			Util.deleteFolderRec(dir, false);	
		} else {
			dir.mkdir();	
		}
		
		// read file names from OWL
		OWLOntology owlMap = OWLIO.loadOntologyFromFile(filename);
		OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
		OWLDataFactory factory = manager.getOWLDataFactory();
		DefaultPrefixManager pm = new DefaultPrefixManager("http://ias.cs.tum.edu/kb/knowrob.owl#");
		pm.setPrefix("knowrob:", "http://ias.cs.tum.edu/kb/knowrob.owl#");
		pm.setPrefix("roboearth:", "http://www.roboearth.org/kb/roboearth.owl#");
		
		OWLDataProperty linkToMapFile   = factory.getOWLDataProperty("roboearth:linkToMapFile", pm);
		OWLDataProperty linkToImageFile = factory.getOWLDataProperty("roboearth:linkToImageFile", pm);
		OWLClass octomapCls = factory.getOWLClass("roboearth:OctoMap", pm);
		
		for(OWLIndividual ind : owlMap.getIndividualsInSignature()) {

			Set<OWLClassExpression> classExpressions = ind.getTypes(owlMap);
			for (OWLClassExpression owlExpr : classExpressions) {
				
				// special treatment for octomaps (extract robot specific 2dmap)
				if (owlExpr.asOWLClass().equals(octomapCls)) {
					
					// download robot SRDL document
					String srdlString = conn.requestRobot(robotUID);
					if (srdlString != null && !srdlString.isEmpty()) {
						OWLOntology srdl = OWLIO.loadOntologyFromString(srdlString);
						
						// request 2d map
						String autoMapFilename = "auto_2d_loc_map";
						String baseLaserLink = "http://ias.cs.tum.edu/kb/amigo.owl#amigo_base_laser"; // TODO auto-extract link name from SRDL?
						
						ArrayList<byte[]> mapBytes;
						mapBytes = conn.request2dMap(Util.getFilenameFromURL(url), srdl, baseLaserLink, autoMapFilename);
						if (Util.writeFile(envDir, autoMapFilename+".pgm", mapBytes.get(0))) {
							outFilenames.add(envDir + File.separator + autoMapFilename+".pgm");	
						}
						if (Util.writeFile(envDir, autoMapFilename+".yaml", mapBytes.get(1))) {
							outFilenames.add(envDir + File.separator + autoMapFilename+".yaml");	
						}						
					} else {
						System.err.println("Error: Couldn't find robot '" + robotUID + "' in RoboEarthDB ");
					}
					
				} 
				
			}

			for(OWLDataPropertyAssertionAxiom dataprop : owlMap.getDataPropertyAssertionAxioms(ind)) {
				if(dataprop.getProperty().equals(linkToMapFile) || dataprop.getProperty().equals(linkToImageFile)) {
					String linkUrl = dataprop.getObject().getLiteral().replaceAll("\\s", "");
					outFileURLs.add(linkUrl);
				}
			}
			
		}
		
		for(int i=0;i<outFileURLs.size();i++) {
			File outfile = conn.requestBinaryFile(new URL(outFileURLs.get(i)), envDir);
			outFilenames.add(outfile.getAbsolutePath());
		}
		
		CommunicationVisApplet.visualizeCommunication("", "Received environment maps "+ Util.getFilenameFromURL(url), "amigo.jpg", "roboearth.png");
		return filename;
		
	}
	
	
	public static String downloadEnvironmentMapFrom(String url, ArrayList<String> outFilenames) throws IOException, ParserConfigurationException, SAXException {

		ArrayList<String> outFileURLs = new ArrayList<String>();
		
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		
		String envDir = Util.tmpDir + Util.getFilenameFromURL(url); 
		String filename = envDir + ".owl";
		
		FileWriter out = new FileWriter(filename);
		out.write(conn.requestEnvironmentFromURL(url));
		out.close();
		
		// read file names from OWL
		OWLOntology owlMap = OWLIO.loadOntologyFromFile(filename);
		OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
		OWLDataFactory factory = manager.getOWLDataFactory();
		DefaultPrefixManager pm = new DefaultPrefixManager("http://ias.cs.tum.edu/kb/knowrob.owl#");
		pm.setPrefix("knowrob:", "http://ias.cs.tum.edu/kb/knowrob.owl#");
		pm.setPrefix("roboearth:", "http://www.roboearth.org/kb/roboearth.owl#");
		
		OWLDataProperty linkToMapFile   = factory.getOWLDataProperty("roboearth:linkToMapFile", pm);
		OWLDataProperty linkToImageFile = factory.getOWLDataProperty("roboearth:linkToImageFile", pm);
		
		for(OWLIndividual ind : owlMap.getIndividualsInSignature()) {
			for(OWLDataPropertyAssertionAxiom dataprop : owlMap.getDataPropertyAssertionAxioms(ind)) {
				
				if(dataprop.getProperty().equals(linkToMapFile) || dataprop.getProperty().equals(linkToImageFile)) {
					String linkUrl = dataprop.getObject().getLiteral().replaceAll("\\s", "");
					outFileURLs.add(linkUrl);
				}
			}
		}
		
		if (outFileURLs.size() > 0) {
			File dir = new File(envDir);
			if (dir.exists()) {
				Util.deleteFolderRec(dir, false);	
			} else {
				dir.mkdir();	
			}

			
			
			for(int i=0;i<outFileURLs.size();i++) {
				File outfile = conn.requestBinaryFile(new URL(outFileURLs.get(i)), envDir);
				outFilenames.add(outfile.getAbsolutePath());
			}
		}
		
		CommunicationVisApplet.visualizeCommunication("", "Received environment maps "+ Util.getFilenameFromURL(url), "amigo.jpg", "roboearth.png");
		return filename;
		
	}
	
	
	
	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Submit new information to the DB
	//
	public static void	submitObject(String owl_filename, String id, String cls, String description) {

		CommunicationVisApplet.visualizeCommunication("Uploading information for '"+id+"' in RoboEarth...", "", "amigo.jpg", "roboearth.png");

		OWLOntology objectOwl = OWLIO.loadOntologyFromFile(owl_filename);
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		boolean res = conn.submitObject(objectOwl, cls, id, description);

		CommunicationVisApplet.visualizeCommunication("", ""+ res, "amigo.jpg", "roboearth.png");
	}


	public static void	submitActionRecipe(String owl_filename, String id, String cls, String description){

		CommunicationVisApplet.visualizeCommunication("Uploading information for '"+id+"' in RoboEarth...", "", "amigo.jpg", "roboearth.png");

		OWLOntology recipeOwl = OWLIO.loadOntologyFromFile(owl_filename);
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		boolean res = conn.submitActionRecipe(recipeOwl, cls, id, description);

		CommunicationVisApplet.visualizeCommunication("", ""+ res, "amigo.jpg", "roboearth.png");
	}


	public static void	submitMap(String owl_filename, String id, String cls, String description){

		CommunicationVisApplet.visualizeCommunication("Uploading information for '"+id+"' in RoboEarth...", "", "amigo.jpg", "roboearth.png");

		OWLOntology environmentOwl = OWLIO.loadOntologyFromFile(owl_filename);
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		boolean res = conn.submitEnvironment(environmentOwl, cls, id, description);

		CommunicationVisApplet.visualizeCommunication("", ""+ res, "amigo.jpg", "roboearth.png");
	}


	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Update information in the DB
	// 

	public static void updateObjectOWL(String owl_filename, String uid, String description) {

		CommunicationVisApplet.visualizeCommunication("Updating information for '"+uid+"' in RoboEarth...", "", "amigo.jpg", "roboearth.png");

//		System.out.println("owl file: " + owl_filename);
		
		//OWLOntology objectOwl = OWLIO.loadOntologyFromFile(owl_filename);
		try {
			BufferedReader reader = new BufferedReader( new FileReader (new File(owl_filename)));
		    String line  = null;
		    StringBuilder stringBuilder = new StringBuilder();
		    String ls = System.getProperty("line.separator");
		    
			while( ( line = reader.readLine() ) != null ) {
			    stringBuilder.append( line );
			    stringBuilder.append( ls );
			}
		    String objectOwl = stringBuilder.toString();

		    
			REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
			boolean res = conn.updateObject(uid, objectOwl, description);
			CommunicationVisApplet.visualizeCommunication("", ""+ res, "amigo.jpg", "roboearth.png");
//		System.out.println("Updating OWL description: id="+uid+"\n\n" + objectOwl.toString());

		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}

	public static void updateActionRecipe(String owl_filename, String id, String description) {

		CommunicationVisApplet.visualizeCommunication("Updating information for '"+id+"' in RoboEarth...", "", "amigo.jpg", "roboearth.png");

		OWLOntology recipeOwl = OWLIO.loadOntologyFromFile(owl_filename);
		REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
		boolean res = conn.updateActionRecipe(id, recipeOwl, description);

		CommunicationVisApplet.visualizeCommunication("", ""+ res, "amigo.jpg", "roboearth.png");
	}


	public static void updateMap(String owl_filename, String id, String description) {

		CommunicationVisApplet.visualizeCommunication("Updating information for '"+id+"' in RoboEarth...", "", "amigo.jpg", "roboearth.png");

		try {
			BufferedReader reader = new BufferedReader( new FileReader (new File(owl_filename)));
		    String line  = null;
		    StringBuilder stringBuilder = new StringBuilder();
		    String ls = System.getProperty("line.separator");
		    
			while( ( line = reader.readLine() ) != null ) {
			    stringBuilder.append( line );
			    stringBuilder.append( ls );
			}
		    String environmentOwl = stringBuilder.toString();

			REConnectionHadoop conn = new REConnectionHadoop(API_KEY);
			boolean res = conn.updateEnvironment(id, environmentOwl, description);
	
			CommunicationVisApplet.visualizeCommunication("", ""+ res, "amigo.jpg", "roboearth.png");

		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}


	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Vision system and world model interface methods
	// 

	public static void publishObjectAsSeenObject(Publisher<ros.pkg.re_msgs.msg.SeenObject> pub, String name, float time, String[] pose) {

		float[] floatpose = new float[16];

		for(int i=0;i<pose.length;i++) {
			floatpose[i]=Float.valueOf(pose[i]);
		}

		publishObjectAsSeenObject(pub, name, time, floatpose);
	}

	public static Publisher<ros.pkg.re_msgs.msg.SeenObject> startObjPublisher() {

		initRos();

		try {
			return(n.advertise("/vslam/seen_objects", new ros.pkg.re_msgs.msg.SeenObject(), 10));
		} catch (RosException e) {
			e.printStackTrace();
		}
		return null;
	}

	public static void stopObjPublisher(Publisher<ros.pkg.re_msgs.msg.SeenObject> pub) {
		pub.shutdown();
	}

	public static void publishObjectAsSeenObject(Publisher<ros.pkg.re_msgs.msg.SeenObject> pub, String name, float time, float[] pose) {

		SeenObject sobj = new SeenObject();
		sobj.name = name;
		sobj.stamp = new Time(time);

		sobj.pose.position.x=pose[3];
		sobj.pose.position.y=pose[7];
		sobj.pose.position.z=pose[11];

		double[] quat = Util.matrixToQuaternion(pose);

		sobj.pose.orientation.w=quat[0];
		sobj.pose.orientation.x=quat[1];
		sobj.pose.orientation.y=quat[2];
		sobj.pose.orientation.z=quat[3];


		// publish as virtual perception
		pub.publish(sobj);
		ros.spinOnce();
		System.err.println("publishing...");

	}

	
	
	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Interface for up- and downloading Cop models
	// 
	
	public static boolean requestCopModel(String object_name) {

		try{

			initRos();

			RoboEarthRetrieveCopModel.Request req = new RoboEarthRetrieveCopModel.Request();
			req.object_name = object_name;

			ServiceClient<RoboEarthRetrieveCopModel.Request, RoboEarthRetrieveCopModel.Response,
			RoboEarthRetrieveCopModel> cl = 
				n.serviceClient("/re_cop_interface/retrieveCopModelByName", 
						new RoboEarthRetrieveCopModel());

			RoboEarthRetrieveCopModel.Response res = cl.call(req);
			// The result is automatically sent to the Cop new signatures topic
			// and since CoP reports all new models to KnowRob anyways, we do
			// not need to take any further actions here.
			cl.shutdown();

			if(res.success==1)
				return true;
			else 
				return false;

		} catch (RosException e) {
			e.printStackTrace();
		} 
		return false;
	}

	/**
	 * 
	 * @param obj_id
	 * @return
	 */
	public static boolean exportCopModel(int obj_id) {

		try{

			initRos();

			RoboEarthExportCopModel.Request req = new RoboEarthExportCopModel.Request();
			req.object_id = obj_id;

			ServiceClient<RoboEarthExportCopModel.Request, RoboEarthExportCopModel.Response, 
			RoboEarthExportCopModel> cl = 
				n.serviceClient("/re_cop_interface/exportCopModel", 
						new RoboEarthExportCopModel());

			RoboEarthExportCopModel.Response res = cl.call(req);
			cl.shutdown();

			if(res.success==1)
				return true;
			else 
				return false;

		} catch (RosException e) {
			e.printStackTrace();
		}
		return false;
	}

	
	
	

	/**
	 * Calls ROS service to download a vSLAM map from RoboEarth
	 * 
	 * @param map_url URL of the vSLAM map in RoboEarth 
	 * @return if all maps were successfully fetched from RoboEarth
	 */
	
	public static boolean updateVslamMaps(String map_url) {

		
		if (map_url != null) {

			try {
				
				initRos();
				
				String uid = Util.getFilenameFromURL(map_url);
				LoadVslamMap.Request req = new LoadVslamMap.Request();
				req.mapUID = uid;

				ServiceClient<LoadVslamMap.Request, LoadVslamMap.Response, LoadVslamMap> cl;
				cl = n.serviceClient("/vslam/load_uid_vmap",	new LoadVslamMap());

				LoadVslamMap.Response res = cl.call(req);
				cl.shutdown();

				if(res.success) 
					return true;

				else 
					return false;

			} catch (RosException e) {
				e.printStackTrace();
			}

		}

		return false;

	}
	
	
	
	
	
	/**
	 * Calls ROS service to enable detection of objects given in the array of
	 * object UIDs
	 * 
	 * @param filenames
	 *            list of objects that shall be detected
	 * @return if all given object models were successfully fetched from
	 *         RoboEarth
	 */
	public static boolean updateObjectModels(String[] filenames) {

		if (filenames != null) {

			try {

				initRos();
				
				ArrayList<String> inputUIDs = new ArrayList<String>();
				DetectObjects.Request req = new DetectObjects.Request();
				req.uids = new ArrayList<String>();

				for (String filename : filenames) {

					// remove path from 
					
					String uid = Util.getFilenameFromURL(filename);
					uid = uid.substring(0, uid.length()-4);
					
					inputUIDs.add(uid);
					req.uids.add(uid);
				}
				
				System.err.println(req.uids.toString());
				
				ServiceClient<DetectObjects.Request, DetectObjects.Response, DetectObjects> cl;
				cl = n.serviceClient("/vslam/load_objects",	new DetectObjects());

				DetectObjects.Response res = cl.call(req);
				cl.shutdown();
				
				
				for (String outputUID : res.detectableUIDs) {
					for (String inputUID : inputUIDs) {
						if (inputUID.equals(outputUID)) {
							inputUIDs.remove(outputUID);
							break;
						}
					}
				}

				if (inputUIDs.size() == 0) {
					return true;
				} else {

					for (String notFoundUID : inputUIDs) {
						System.out.println("Warning: object model '"
								+ notFoundUID + "' couldn't be found!");
					}

					return false;
				}

			} catch (RosException e) {
				e.printStackTrace();
			}

		}

		return false;

	}
	
	
	//
	// ugly method for conveniently resetting the database content to the state
	// before the update
	//
	public static void resetDBcontent() {
		 
		System.out.println("Updating bedrecmodel");
		updateObjectOWL("/home/tenorth/re-obj-models/bed/bed.owl", "bedrecmodel.bedrecmodel", "");
		
		System.out.println("Updating bottlerecmodel");
		updateObjectOWL("/home/tenorth/re-obj-models/bottle/bottle.owl", "bottlerecmodel.bottlerecmodel", "");
		
		System.out.println("Updating expedit");
		updateObjectOWL("/home/tenorth/re-obj-models/expedit/expedit.owl", "cabinet.ikeaexpedit2x4", "");

		//System.out.println("Updating ks map");
		//updateMap("/home/tenorth/re-obj-models/semantic-map-ks.owl", "semanticenvironmentmap.semanticenvironmentmap7635", "");
		
		//System.out.println("Updating fmi map");
		//updateMap("/home/tenorth/re-obj-models/semantic-map-fmi.owl", "semanticenvironmentmap.semanticenvironmentmap7398", "");
	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// SAX SPARQL reader methods
	// 
	

	public class SparqlObjReader extends DefaultHandler {

		protected boolean uri = false;
		public void startElement(String uri, String name, String qName, Attributes attrs) {
			if(qName.equals("uri")) {
				this.uri = true;
			}
		}

		@Override
		public void characters( char[] ch, int start, int length ) {

			if(uri) {
				obj_urls.add(new String(ch).substring(start, start+length));
				this.uri=false;
			}
		}
	}
	
	public class SparqlRecipeReader extends DefaultHandler {

		protected boolean uri = false;
		public void startElement(String uri, String name, String qName, Attributes attrs) {
			if(qName.equals("uri")) {
				this.uri = true;
			}
		}

		@Override
		public void characters( char[] ch, int start, int length ) {

			if(uri) {
				act_urls.add(new String(ch).substring(start, start+length));
				this.uri=false;
			}
		}
	}

	public class SparqlEnvReader extends DefaultHandler {

		protected boolean uri = false;
		public void startElement(String uri, String name, String qName, Attributes attrs) {
			if(qName.equals("uri")) {
				this.uri = true;
			}
		}

		@Override
		public void characters( char[] ch, int start, int length ) {

			if(uri) {
				env_urls.add(new String(ch).substring(start, start+length));
				this.uri=false;
			}
		}
	}
}
