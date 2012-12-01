/* \file Util.java
 * \brief Utility class
 *
 * The Utility class provides useful helper methods.
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
 * \author Moritz Tenorth
 * \version 1.0
 * \date 2010
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 */
package roboearth.wp5.util;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.AddImport;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLImportsDeclaration;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;

import roboearth.wp5.conn.REConnectionHadoop;
import roboearth.wp5.owl.IRIDepot;
import ros.pkg.re_msgs.msg.RosFile;

/**
 * Util is a utility class that provides methods to create/extract zip files, copy files 
 * to and from a remote computer and retrieving the IP/URI of the computer offering a 
 * given ROS service.
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 * @author Alexander Perzylo, perzylo@cs.tum.edu
 * 
 */
public class Util {

	public final static String modelDir;
	public final static String tmpDir;
	public final static String re_commDir;
	static {
		re_commDir = Util.getLocalRosPackagePath("re_comm");
		if (re_commDir == null) {
			System.out.println("Couldn't find local path of re_comm package. "+
					"(Is environmental variable ROS_ROOT set correctly?)");
			System.exit(1);
		}
		tmpDir = re_commDir + "tmp/";
		modelDir = re_commDir + "models/";
	}
	
	/**
	 * Reads the IP address/URI of the computer offering a service.
	 * 
	 * @return URI of the computer
	 */
	public static String getURIforService(String service) {

		try {

			String rosservice = System.getenv().get("ROS_ROOT")+"/bin/rosservice";
			Process p = new ProcessBuilder( rosservice, "uri", service ).start(); 

			StringBuilder sb = new StringBuilder();
			String line;

			try {
				BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream(), "UTF-8"));
				while ((line = reader.readLine()) != null) {
					sb.append(line).append("\n");
				}
			} finally {
				p.getInputStream().close();
			}
			return sb.toString();

		} catch (IOException e) {
			e.printStackTrace();
		}
		return "";

	}

	/**
	 * Copies a remote file via scp into a local directory.
	 * 
	 * @param remotefile Absolute path and filename on the remote computer
	 * @param remoteHost Remote host to copy from
	 * @param localdir Local directory to copy into
	 */
	public static boolean sshCopyFromRemote(String remotefile, String remoteHost, String localdir) {

		boolean ok = false;

		// get local hostname
		String uri = Util.getURIforService("/re_comm/set_object");
		int index = uri.lastIndexOf(":");
		String localHostname = (index > 9) ? uri.substring(9, index):uri.substring(9);
		
		try {	

			Process p;
			if (remoteHost.equals("localhost") || localHostname.equals(remoteHost)) {
				p = new ProcessBuilder( "/bin/cp", remotefile, localdir ).start();
			} else {
				p = new ProcessBuilder( "/usr/bin/scp", remoteHost+":"+remotefile, localdir ).start();	
			}

			try {
				BufferedReader reader = new BufferedReader(new InputStreamReader(p.getErrorStream(), "UTF-8"));

				String line;
				while ((line = reader.readLine()) != null) {
					System.err.println(line);
				}

				ok = true;

			} finally {
				p.getInputStream().close();
			}

		} catch (Exception e) {
			e.printStackTrace(System.err);
			System.exit(2);
		}

		return ok;

	}

	/**
	 * Copies a local file via scp into a remote directory on the computer hostname.
	 * 
	 * @param localfile Absolute path and filename on the local host
	 * @param remoteHost remote host
	 * @param remoteDir directory on remote host to copy into
	 */
	public static boolean sshCopyToRemote(String localfile, String remoteHost, String remotedir) {

		boolean ok = false;

		// get local hostname
		String uri = Util.getURIforService("/re_comm/get_object");
		int index = uri.lastIndexOf(":");
		String localHostname = (index > 9) ? uri.substring(9, index):uri.substring(9);
		
		try {	
			String line;

			Process copy_file;
			if (remoteHost.equals("localhost") || localHostname.equals(remoteHost)) {
				copy_file = new ProcessBuilder( "/bin/cp", localfile, remotedir ).start();
			} else {
				copy_file = new ProcessBuilder( "/usr/bin/scp", localfile, remoteHost+":"+remotedir ).start();	
			}
			
			try {
				BufferedReader errors = new BufferedReader(new InputStreamReader(copy_file.getErrorStream(), "UTF-8"));

				while ((line = errors.readLine()) != null) {
					System.err.println(line);
				}

				ok = true;

			} finally {
				copy_file.getInputStream().close();
			}
		}
		catch (IOException e) {
			e.printStackTrace(System.err);
			System.exit(2);
		}

		return ok;

	}

	/**
	 * Searches for the absolute path of the given ROS package on a given host.
	 * 
	 * @param hostname host name, which is to be searched
	 * @param rosPackageName name of ROS package to search for
	 * @return Absolute path of the given ROS package on the remote host, 
	 * or null if the host or package wasn't found
	 */
	public static String getRemoteRosPackagePath(String hostname, String rosPackageName) {

		String dir = null;

		try {	

			// try to find package using rospack
			String line;
			Process get_path = new ProcessBuilder( "/usr/bin/ssh", hostname, "bash -i -c 'rospack find "+rosPackageName+"'" ).start();

			try {
				BufferedReader pathreader = new BufferedReader(new InputStreamReader(get_path.getInputStream(), "UTF-8"));

				if( (line = pathreader.readLine()) != null) {
					dir = line+File.separator;
				}
			} finally {
				if (get_path != null) {
					get_path.getInputStream().close();	
				}
			}

		} catch (IOException e) {
			e.printStackTrace(System.err);
		}

		return dir;

	}

	/**
	 * Searches for the absolute path of the given ROS package on the local host.
	 * 
	 * @param rosPackageName name of ROS package to search for
	 * @return Absolute path of the given ROS package or null if the package wasn't found
	 */
	public static String getLocalRosPackagePath(String rosPackageName) {

		String dir = null;

		try {	

			// try to find package using rospack
			String line;
			String rospack = System.getenv().get("ROS_ROOT")+"/bin/rospack";
			Process get_path = new ProcessBuilder( rospack, "find", rosPackageName).start();

			try {
				BufferedReader pathreader = new BufferedReader(new InputStreamReader(get_path.getInputStream(), "UTF-8"));

				if( (line = pathreader.readLine()) != null) {
					dir = line+File.separator;
				}
			} finally {
				if (get_path != null) {
					get_path.getInputStream().close();	
				}
			}

		} catch (IOException e) {
			e.printStackTrace(System.err);
		}

		return dir;

	}
	
	/**
	 * Extracts the CLASS from an UID starting from index 0 to the index of 
	 * the last occurrence of '.' (excluded).
	 * 
	 * @param uid a unique identifier
	 * @return the class part of the uid
	 */
	public static String getClassFromUID(String uid) {

		String cls = null;

		if (uid != null) {
			int index = uid.lastIndexOf(".");
			if (index > 0) {
				cls = uid.substring(0,index);	
			} else {
				cls = "";
			}
		}

		return cls;

	}

	/**
	 * Extracts the ID from an UID (starting from the index of 
	 * the last occurrence of '.' + 1)
	 * 
	 * @param uid a unique identifier
	 * @return the last part of the uid (also called the id)
	 */
	public static String getIDFromUID(String uid) {

		String id = null;

		if (uid != null) {
			id = uid.substring(uid.lastIndexOf(".")+1);	
		}

		return id;

	}

	/**
	 * Creates a ZIP archive from a list of files.
	 * 
	 * @param filenames List of files to be zipped
	 * @param target File name of the resulting zip archive
	 */
	public static void createZipFromFiles(Set<String> filenames, String target) {

		byte[] buffer = new byte[1024];
		try {

			int size;
			FileInputStream in;
			ZipOutputStream out = new ZipOutputStream(new FileOutputStream(target));

			// add files
			for (String f : filenames) {

				in = new FileInputStream(f);
				out.putNextEntry(new ZipEntry(new File(f).getName()));

				while ((size = in.read(buffer)) > 0) {
					out.write(buffer, 0, size);
				}

				out.closeEntry();
				in.close();
			}
			out.close();

		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	/**
	 * Extracts files from a ZIP archive into a folder.
	 * 
	 * @param filename Filename of the ZIP file to be extracted
	 * @param targetFolder Target directory into which the files are to be extracted
	 */
	public static boolean extractZipFile(String filename, String targetFolder) {

		boolean ok = false;
		
		try {

			byte[] buffer = new byte[1024];
			ZipInputStream in = new ZipInputStream(new FileInputStream(filename));
			ZipEntry zipentry = in.getNextEntry();

			while (zipentry != null) {

				File newFile = new File(zipentry.getName());
				String directory = newFile.getParent();

				if(directory == null) {
					if(newFile.isDirectory())
						break;
				}

				FileOutputStream out = new FileOutputStream(targetFolder+zipentry.getName());

				int n;
				while ((n = in.read(buffer, 0, 1024)) > -1)
					out.write(buffer, 0, n);

				out.close(); 
				in.closeEntry();
				zipentry = in.getNextEntry();
			}
			in.close();
			ok = true;
		} catch (Exception e) {
			e.printStackTrace();
		}

		return ok;
		
	}

	/**
	 * Retrieves the simple file name from a long file name containing the path 
	 * by searching for the last index of the system dependent file separator and 
	 * returning a substring starting from that index + 1. If no file separator 
	 * was found, the input string gets returned.  
	 * 
	 * @param longFilename long file name
	 * @return simple file name (without path)
	 */
	public static String getSimpleFilename(String longFilename) {

		if (longFilename == null) {
			return null;
		} else {
			return longFilename.substring(longFilename.lastIndexOf(File.separator)+1, longFilename.length());	
		}

	}

	/**
	 * Retrieves the last part of the given URL that starts at the index of the 
	 * last occurrence of '/' + 1. If no '/' was found, the input string gets 
	 * returned.
	 * @param url a URL
	 * @return the part of the URL after the last occurrence of '/'
	 */
	public static String getFilenameFromURL(String url) {

		if (url == null) {
			return null;
		} else {
			return url.substring(url.lastIndexOf("/")+1, url.length());	
		}

	}

	/**
	 * This function will recursively delete directories and files.
	 * 
	 * @param path Directory to be deleted
	 * @param alsoDeleteGivenFolder Flag, indicating whether the directory that 
	 * was passed in as first argument shall be deleted along with its contents 
	 * or not. 
	 * @return <tt>true</tt> if deletion was successfully completed,<br>
	 * <tt>false</tt> otherwise. Parts of the given folder might already be deleted. 
	 */
	public static boolean deleteFolderRec(File path, boolean alsoDeleteGivenFolder) {

		boolean ok; 

		if (path.exists()) {
			ok = true;
			if (path.isDirectory()) {
				File[] files = path.listFiles();
				for (int i=0; i<files.length; i++) {
					if(files[i].isDirectory()) {
						deleteFolderRec(files[i], true);
					} else {
						files[i].delete();
					}
				}
				if (alsoDeleteGivenFolder) {
					ok = ok && path.delete();
				}
			}
		} else {
			ok = false;
		}

		return ok;
	}

	public static double[] matrixToQuaternion(float[] m) {

		double[] res = new double[4];

		// w
		res[0] = Math.sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;

		double w4 = (4.0 * res[0]);
		res[1] = (m[9] - m[6]) / w4 ; // x
		res[2] = (m[2] - m[8]) / w4 ; // y
		res[3] = (m[4] - m[1]) / w4 ; // z

		return res;
	}

	/**
	 * Writes a file contained in a re_msgs/RosFile message to the given local
	 * path.
	 * 
	 * @param targetPath
	 *            path to store file in
	 * @param content
	 *            the RosFile message
	 */
	public static boolean writeRosFile(String targetPath, RosFile content) {

		boolean ok = false;

		if (content != null && targetPath != null && targetPath.length() > 0) {
			ok = writeFile(targetPath, content.name, content.data);
		}

		return ok;

	}
	
	public static boolean writeFile(String targetPath, String filename, byte[] content) {
		
		boolean ok = false;

		if (content != null && targetPath != null && targetPath.length() > 0 
				&& filename != null && filename.length() > 0) {

			BufferedOutputStream bos = null;

			if (!targetPath.endsWith(File.separator)) {
				targetPath += File.separator;
			}
			
			try {
				FileOutputStream fos;
				fos = new FileOutputStream(new File(targetPath+filename));
				bos = new BufferedOutputStream(fos);
				bos.write(content);
				bos.flush();
				ok = true;
			} catch (Exception e) {
				e.printStackTrace();
			} finally {
				if (bos != null) {
					try {
						bos.close();
					} catch (Exception e) {
					}
				}
			}
			
		}

		return ok;
		
	}
	
	/**
	 * Main method for testing purposes.
	 * @param args arguments get ignored
	 */
	public static void main(String[] args) {

		if (args.length < 1 || args.length > 1) {
			System.out.println("\nUsage:\n" +
					"This test program assumes exactly one argument, which " +
					"is the API key for the RoboEarthDB interface.\nYou " +
					"may get yours at:\n" +
					"http://roboearth.informatik.uni-stuttgart.de\n" +
					"If no valid API key is given, reading from the " +
					"RoboEarthDB is still possible, but submitting data " +
					"will fail.\n" +
					"\nExample:\n" +
					"rosrun re_comm test XYZ1234567890XYZ\n");
			return;
		}
		
		String key = args[0];

		REConnectionHadoop re = new REConnectionHadoop(key);

		// create dummy ontology
		OWLOntology ontology = null;

		try{
			// Create ontology manager and data factory
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(IRIDepot.ROBOEARTH));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(IRIDepot.KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);
		} catch(OWLOntologyCreationException e) {e.printStackTrace();}

		//System.out.println(OWLIO.saveOntologytoString(ontology));

		long sleepTime = 200;

		try {

			String id ="recipe1";
			String cls = "testrecipe";
			String uid = cls+"."+id;

			// test action recipes
			System.out.println("\nTesting recipes:");
			re.deleteActionRecipe(uid);
			Thread.sleep(sleepTime);
			re.submitActionRecipe(ontology, cls, id, "submitted recipe");
			Thread.sleep(sleepTime);
			re.updateActionRecipe(uid, ontology, "updated recipe");
			Thread.sleep(sleepTime);
			re.requestActionRecipe(uid);
			Thread.sleep(sleepTime);
			re.deleteActionRecipe(uid);
			Thread.sleep(sleepTime);

			id = "object1";
			cls = "testobject";
			uid = cls+"."+id;

			// test object models (new objects API)			
			System.out.println("\nTesting objects:");
			re.deleteObject(uid);
			Thread.sleep(sleepTime);
			re.submitObject(ontology, cls, id, "submitted object", null);
			Thread.sleep(sleepTime);
			re.updateObject(uid, ontology, "updated object description");
			Thread.sleep(sleepTime);
			
			ArrayList<String> outFilenames = new ArrayList<String>();
			ArrayList<String> outFileURLs = new ArrayList<String>();
			String res = re.requestObject(uid, outFilenames, outFileURLs);
			if (res != null) {
				for (String filename : outFilenames) {
					re.requestObjectBinaryFile(uid, filename);
				}
			}
			Thread.sleep(sleepTime);
			re.deleteObject(uid);
			Thread.sleep(sleepTime);

			id = "map1";
			cls = "testenv";
			uid = cls+"."+id;

			// test environments
			System.out.println("\nTesting environments:");
			re.deleteEnvironment(uid);
			Thread.sleep(sleepTime);
			re.submitEnvironment(ontology, cls, id, "submitted environemnt");
			Thread.sleep(sleepTime);
			re.updateEnvironment(uid, ontology, "updated environment");
			Thread.sleep(sleepTime);
			re.requestEnvironment(uid);
			Thread.sleep(sleepTime);
			re.deleteEnvironment(uid);
			System.out.println();

		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		String service = "/re_comm/get_action_recipe";
		String uri = Util.getURIforService(service);
		if (uri == null || uri.length()==0) {
			uri = "(re_comm must be running)";
		}
		System.out.println("URI for service '"+service+"':\n"+uri);
		
	}

}
