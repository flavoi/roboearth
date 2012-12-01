/* \file REConnectionMySQL.java
 * \brief Connection to MySQL based RoboEarthDB 
 *
 * The connection class for a MySQL based RoboEarthDB installation.
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
package roboearth.wp5.conn;

import java.io.File;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;

import org.semanticweb.owlapi.model.OWLOntology;

import roboearth.wp5.owl.OWLIO;


/**
 * 
 * The REConnection class provides means to interact with the
 * RoboEarth web interface. It contains methods to submit and retrieve
 * all supported kinds of information relevant to the RoboEarth project. 
 * 
 * MySQL-based implementation by Moritz Tenorth
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */
public class REConnectionMySQL implements REInterface {


	/**
	 * Host name of the RE MySQL database
	 */
	public final static String roboEarthMysqlHost = "atradig131.informatik.tu-muenchen.de";

	/**
	 * Host name of the RE MySQL database
	 */
	public final static String roboEarthMysqlDB = "roboearth";

	/**
	 * Host name of the RE MySQL database
	 */
	public final static String roboEarthMysqlUser = "roboearth";

	/**
	 * Host name of the RE MySQL database
	 */
	public final static String roboEarthMysqlPassword = "RoboEarth2010";


	protected Connection conn = null;


	public REConnectionMySQL() {

		try {
			Class.forName("com.mysql.jdbc.Driver").newInstance();
			conn = DriverManager.getConnection("jdbc:mysql://"+roboEarthMysqlHost+"/"+
					roboEarthMysqlDB+"?" +
					"user="+roboEarthMysqlUser+
					"&password="+roboEarthMysqlPassword);

		} catch (SQLException ex) {
			// handle any errors
			System.out.println("SQLException: " + ex.getMessage());
			System.out.println("SQLState: " + ex.getSQLState());
			System.out.println("VendorError: " + ex.getErrorCode());

		} catch (Exception ex) {
			ex.printStackTrace();
		}

	}

	
	
	



	@Override
	public String requestActionRecipe(String id) {

		String recipe = null;

		if (id != null && id.length()>0) {

			String query = "SELECT title, tags, description, recipe from actionrecipes where title='"+id+"'";
			String owlDescription="";

			Statement stmt = null;
			ResultSet rs = null;

			try {
				stmt = conn.createStatement();
				rs = stmt.executeQuery(query);


				if (!rs.next()) {
					System.out.println("No recipe found.");
				} else {

					// create OWL description
					owlDescription = cleanOWLString(rs.getString(4));
					recipe = owlDescription;
				}
			}
			catch (SQLException ex){
				// handle any errors
				System.out.println("SQLException: " + ex.getMessage());
				System.out.println("SQLState: " + ex.getSQLState());
				System.out.println("VendorError: " + ex.getErrorCode());
			}
			finally {

				if (rs != null) {
					try {
						rs.close();
					} catch (SQLException sqlEx) { } // ignore

					rs = null;
				}

				if (stmt != null) {
					try {
						stmt.close();
					} catch (SQLException sqlEx) { } // ignore

					stmt = null;
				}
			}


		}

		return recipe;

	}

	

	public String requestEnvironment(String id) {

		String map = null;

		if (id != null && id.length()>0) {

			String query = "SELECT title, tags, description, map from maps where title='"+id+"'";
			String owlDescription="";

			Statement stmt = null;
			ResultSet rs = null;

			try {
				stmt = conn.createStatement();
				rs = stmt.executeQuery(query);


				if (!rs.next()) {
					System.out.println("No environment found.");
				} else {

					// create OWL description
					owlDescription = cleanOWLString(rs.getString(4));
					map = owlDescription;
				}
			}
			catch (SQLException ex){
				// handle any errors
				System.out.println("SQLException: " + ex.getMessage());
				System.out.println("SQLState: " + ex.getSQLState());
				System.out.println("VendorError: " + ex.getErrorCode());
			}
			finally {

				if (rs != null) {
					try {
						rs.close();
					} catch (SQLException sqlEx) { } // ignore

					rs = null;
				}

				if (stmt != null) {
					try {
						stmt.close();
					} catch (SQLException sqlEx) { } // ignore

					stmt = null;
				}
			}


		}

		return map;

	}


	@Override
	public boolean submitActionRecipe(OWLOntology actionRecipe, String id, String tags, String description) {

		boolean ok = false;

		if (actionRecipe != null && id != null && tags != null && description != null) {

			try{
				PreparedStatement stmt = conn.prepareStatement("INSERT INTO actionrecipes(title, tags, description, recipe)" +
				" VALUES(?, ?, ?, ?)");

				stmt.setString(1, id);
				stmt.setString(2, tags);
				stmt.setString(3, description);

				String owlData = OWLIO.saveOntologyToString(actionRecipe, actionRecipe.getOWLOntologyManager().getOntologyFormat(actionRecipe));
				stmt.setString(4, owlData);

				stmt.executeUpdate();

				ok=true;

			} catch (SQLException e) {
				e.printStackTrace();
			}
		}

		return ok;

	}

	@Override
	public boolean submitEnvironment(OWLOntology map, String id, String tags, String description) {

		boolean ok = false;

		if (map != null && id != null && tags != null && description != null) {


			try{
				PreparedStatement stmt = conn.prepareStatement("INSERT INTO maps(title, tags, description, map)" +
				" VALUES(?, ?, ?, ?)");

				stmt.setString(1, id);
				stmt.setString(2, tags);
				stmt.setString(3, description);

				String owlData = OWLIO.saveOntologyToString(map, map.getOWLOntologyManager().getOntologyFormat(map));
				stmt.setString(4, owlData);

				stmt.executeUpdate();

				ok=true;

			} catch (SQLException e) {
				e.printStackTrace();
			}
		}

		return ok;

	}
	
	/**
	 * Replaces HTML entities within the given string with the characters they
	 * represent, e.g. the HTML entity <xmp>&lt;</xmp> will be replaced with the symbol < 
	 * @param owlData the owl data string to be cleaned
	 * @return the cleaned string
	 */
	public static String cleanOWLString(String owlData) {

		return owlData.replaceAll("&lt;", "<").replaceAll("&gt;", ">").replaceAll("&quot;", "\"").replaceAll("\t", "").trim();

	}

	/**
	 * Main method for testing purposes only. 
	 */
	public static void main(String[] args) {

//		REConnectionMySQL re = new REConnectionMySQL();
//
//		// create dummy ontology
//		OWLOntology ontology = null;
//
//		try{
//			// Create ontology manager and data factory
//			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
//			OWLDataFactory factory = manager.getOWLDataFactory();
//
//			// Create empty OWL ontology
//			ontology = manager.createOntology(IRI.create(IRIDepot.ROBOEARTH));
//			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());
//
//			// Import KnowRob ontology
//			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(IRIDepot.KNOWROB));
//			AddImport addImp = new AddImport(ontology,oid);
//			manager.applyChange(addImp);
//		} catch(OWLOntologyCreationException e) { }
//
//		// test action recipes
//		re.submitActionRecipe(ontology, "test_recipe", "bla, bla", "tasty action recipe");
//		OWLOntology res = re.requestActionRecipe("test_recipe");
//		System.out.println(OWLIO.saveOntologytoString(res, res.getOWLOntologyManager().getOntologyFormat(res)));
//
//
//		// test object models
//		re.submitObject(ontology, "Cup", "Cup", new File("icub.jpg"), new File("icub.jpg"));
//		res = re.requestObject("Cup", "out1.jpg", "out2.jpg");
//		System.out.println(OWLIO.saveOntologytoString(res, res.getOWLOntologyManager().getOntologyFormat(res)));
//
//		// test maps
//		re.submitEnvironment(ontology, "test_map", "bla, bla", "large environment map");
//		res = re.requestEnvironment("test_map");
//		System.out.println(OWLIO.saveOntologytoString(res, res.getOWLOntologyManager().getOntologyFormat(res)));


	}




	@Override
	public boolean deleteActionRecipe(String uid) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean deleteEnvironment(String uid) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean deleteEnvironmentBinaryFile(String uid, String filename) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean deleteObject(String uid) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean deleteObjectBinaryFile(String uid, String filename) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public String queryActionRecipeDB(String seRQLquery) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public String queryEnvironmentDB(String seRQLquery) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public String queryObjectDB(String seRQLquery) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public String requestEnvironment(String uid,
			ArrayList<String> outFilenames, ArrayList<String> outFileURLs) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public File requestEnvironmentBinaryFile(String objectUID, String filename,
			String targetPath) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public byte[] requestEnvironmentBinaryFile(String objectUID, String filename) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public String requestObject(String uid, ArrayList<String> outFilenames,
			ArrayList<String> outFileURLs) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public File requestObjectBinaryFile(String objectUID, String filename,
			String targetPath) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public byte[] requestObjectBinaryFile(String objectUID, String filename) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public String[] searchActionRecipes(String searchID,
			ArrayList<String> outUIDs) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public String[] searchEnvironments(String searchID,
			ArrayList<String> outUIDs,
			ArrayList<ArrayList<String>> outFilenames,
			ArrayList<ArrayList<String>> outFileURLs) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public String[] searchObjects(String searchID, ArrayList<String> outUIDs,
			ArrayList<ArrayList<String>> outFilenames,
			ArrayList<ArrayList<String>> outFileURLs) {
		// TODO Auto-generated method stub
		return null;
	}




	@Override
	public boolean submitEnvironment(OWLOntology env, String cls, String id,
			String description, ArrayList<File> binaryFiles) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitEnvironment(OWLOntology env, String cls, String id,
			String description, ArrayList<byte[]> binaryData,
			ArrayList<String> filenames) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitEnvironmentBinaryFile(String uid, File file) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitEnvironmentBinaryFile(String uid, byte[] data,
			String filename) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitObject(OWLOntology objectOwl, String cls, String id,
			String description) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitObject(OWLOntology objectOwl, String cls, String id,
			String description, ArrayList<File> binaryFiles) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitObject(OWLOntology objectOwl, String cls, String id,
			String description, ArrayList<byte[]> binaryData,
			ArrayList<String> filenames) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitObjectBinaryFile(String uid, File file) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean submitObjectBinaryFile(String uid, byte[] data,
			String filename) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean updateActionRecipe(String uid, OWLOntology actionRecipe,
			String description) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean updateEnvironment(String uid, OWLOntology env,
			String description) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean updateEnvironmentBinaryFile(String uid, File file) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean updateEnvironmentBinaryFile(String uid, byte[] data,
			String filename) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean updateObject(String uid, OWLOntology objectOwl,
			String description) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean updateObjectBinaryFile(String uid, File file) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public boolean updateObjectBinaryFile(String uid, byte[] data,
			String filename) {
		// TODO Auto-generated method stub
		return false;
	}




	@Override
	public ArrayList<byte[]> request2dMap(String envUid, OWLOntology srdl,
			String baseScannerLink, String simpleMapNameWithoutExt) {
		// TODO Auto-generated method stub
		return null;
	}


	

	@Override
	public ArrayList<byte[]> requestProjected2dMap(String envUid, double minZ,
			double maxZ, String simpleMapNameWithoutExt) {
		// TODO Auto-generated method stub
		return null;
	}







	@Override
	public boolean updateObject(String uid, String objectOwl, String description) {
		// TODO Auto-generated method stub
		return false;
	}







	@Override
	public String requestRobot(String uid) {
		// TODO Auto-generated method stub
		return null;
	}


}
