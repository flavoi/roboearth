/* \file REConnectionHadoop.java
 * \brief Connection to Hadoop based RoboEarthDB
 *
 * The connection class for a Hadoop based RoboEarthDB installation.
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

import static argo.jdom.JsonNodeFactories.aJsonField;
import static argo.jdom.JsonNodeFactories.aJsonObject;
import static argo.jdom.JsonNodeFactories.aJsonString;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLConnection;
import java.util.AbstractList;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.StringTokenizer;

import org.apache.http.Header;
import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpDelete;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.client.methods.HttpPut;
import org.apache.http.entity.StringEntity;
import org.apache.http.entity.mime.MultipartEntity;
import org.apache.http.entity.mime.content.InputStreamBody;
import org.apache.http.entity.mime.content.StringBody;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicHeader;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;
import org.apache.http.util.EntityUtils;
import org.semanticweb.owlapi.model.OWLOntology;

import roboearth.wp5.owl.Inference;
import roboearth.wp5.owl.OWLIO;
import roboearth.wp5.util.Util;
import argo.format.JsonFormatter;
import argo.format.PrettyJsonFormatter;
import argo.jdom.JdomParser;
import argo.jdom.JsonNode;
import argo.jdom.JsonNodeSelector;
import argo.jdom.JsonNodeSelectors;
import argo.jdom.JsonRootNode;
import argo.jdom.JsonStringNode;
import argo.saj.InvalidSyntaxException;

/**
 * 
 * The REConnectionHadoop class provides means to interact with the
 * RoboEarth DB provided by an Apache Hadoop based implementation.
 * It contains methods to submit and retrieve all supported kinds of
 * information relevant to the RoboEarth project. 
 * 
 * @author Alexander Perzylo, perzylo@cs.tum.edu
 *
 */
public class REConnectionHadoop implements REInterface {

	/**
	 * Base URL to the RoboEarth web interface
	 */
	public final static String roboEarthBaseURL = "http://api.roboearth.org";

	/**
	 * API key for the REST interface to the RoboEarth DB
	 */
	private final String apiKey;

	/**
	 * URL suffix for requesting information about an object
	 */
	public final static String requestObjectSuffix = "/api/object/";

	/**
	 * URL suffix for requesting an action recipe
	 */
	public final static String requestActionRecipeSuffix = "/api/recipe/";

	/**
	 * URL suffix for requesting an environment
	 */
	public final static String requestEnvironmentSuffix = "/api/environment/";

	/**
	 * URL suffix for requesting a robot
	 */
	public final static String requestRobotSuffix = "/api/robot/";	
	
	/**
	 * URL suffix for requesting binary files related to objects
	 */
	public final static String requestObjectBinariesSuffix = "/data/objects/";

	/**
	 * URL suffix for requesting binary files related to objects
	 */
	public final static String requestEnvironmentBinariesSuffix = "/data/environments/";
	
	/**
	 * URL suffix for submitting information about an object
	 */
	public final static String submitObjectSuffix = "/api/object";

	/**
	 * URL suffix for submitting an action recipe
	 */
	public final static String submitActionRecipeSuffix = "/api/recipe";

	/**
	 * URL suffix for submitting an environment
	 */
	public final static String submitEnvironmentSuffix = "/api/environment";

	/**
	 * URL suffix for submitting binary files related to objects
	 */
	public final static String submitObjectBinariesSuffix = "/api/binary/objects/";
	
	/**
	 * URL suffix for submitting binary files related to environments
	 */
	public final static String submitEnvironmentBinariesSuffix = "/api/binary/environments/";
	
	/**
	 * URL suffix for querying the semantic DB
	 */
	public final static String querySuffix = "/api/serql";

	/**
	 * URL suffix for requesting 2d map extraction from 3d octomaps
	 */
	public final static String mapExtractorSuffix = "/api/service/3dto2dmap";
	
	/**
	 * JSON parser used to extract data fields from HTTP responses from the RoboEarth DB
	 */
	private final static JdomParser json_parser = new JdomParser();

	/**
	 * JSON formatter that creates string representations of JSON objects 
	 */
	private final static JsonFormatter JSON_FORMATTER = new PrettyJsonFormatter();

	/**
	 * Connection timeout in milliseconds
	 */
	public final static int timoutInMs = 30000;
	
	@Deprecated
	private final static String fileIdObjImage = "image";
	@Deprecated
	private final static String fileIdObjModel = "model";
	@Deprecated
	private final static String fileIdObjArticModel = "articulationModel";


	/**
	 * Creates a RoboEarth connection and assigns an API key for authentication
	 * at the RoboEarth DB. You may get your API key at:
	 * http://roboearth.informatik.uni-stuttgart.de
	 * 
	 * You can also pass in a null reference. Then it would be possible to 
	 * request data from the RoboEarth DB, but submitting data will fail.
	 * 
	 * @param apiKey API key for the RoboEarth DB
	 */
	public REConnectionHadoop(String apiKey) {

		this.apiKey = apiKey;

	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download action recipes from the DB
	//
	
	@Override
	public String requestActionRecipe(String uid) {
		if (uid != null) {
			return requestActionRecipeFromURL(roboEarthBaseURL+requestActionRecipeSuffix+uid);
		} else {
			System.out.println("\ndownload recipe '"+uid+"': parameter error (null pointer)");
		}
		return null;
	}

	/**
	 * Requests an action recipe.
	 * 
	 * @param url the URL of the requested action recipe
	 * @return String object containing OWL description of an action recipe
	 */
	public String requestActionRecipeFromURL(String url) {

		//System.out.print("\ndownload recipe '"+Util.getFilenameFromURL(url)+"': ");
		
		String responseBody = "";
		if (url != null) {

			responseBody = requestData(url);
			if (responseBody!= null && (responseBody.startsWith("[") || responseBody.startsWith("{"))) {

				try {
					// Create a list of returned recipes and their time stamps, ...
					RecipeList recipes = new RecipeList(responseBody);
					responseBody = recipes.get(0);

				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}

			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return responseBody;
	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download environment maps from the DB
	//
	

	public String requestEnvironment(String uid) {
		return requestEnvironment(uid, null, null);
	}

	/**
	 * Requests information about an environment. The names and URLs of binary 
	 * files related to the requested environment will be put into the given 
	 * lists. Any element that might be in the lists before the method call 
	 * will be removed first. Both list parameters may be null. In this case 
	 * only the OWL encoded description is requested.
	 * 
	 * @param uid the environment's identifier
	 * @param outFilenames this list will be used to return the names
	 * of associated files
	 * @param outFileURLs this list will be used to return the URLs
	 * of associated files
	 * @return String object containing OWL description of environmental data
	 */
	public String requestEnvironment(String uid, ArrayList<String> outFilenames, ArrayList<String> outFileURLs) {
		if (uid != null) {
			return requestEnvironmentFromURL(roboEarthBaseURL+requestEnvironmentSuffix+uid, outFilenames, outFileURLs);
		} else {
			System.out.println("\ndownload environment '"+uid+"': parameter error (null pointer)");
			return null;
		}
	}
	
	/**
	 * Requests a description of an environment.
	 * 
	 * @param url the URL of the requested environment
	 * @return String object containing OWL description of environmental data
	 */
	public String requestEnvironmentFromURL(String url) {
		return requestEnvironmentFromURL(url, null, null);
	}
	
	/**
	 * Requests information about an environment. The names and URLs of binary 
	 * files related to the environment represented by its own URL will be put 
	 * into the given lists. Any element that might be in the lists before 
	 * the method call will be removed first. Both list parameters may be 
	 * null. In this case only the OWL encoded description is requested.
	 * 
	 * @param url the environment's URL
	 * @param outFilenames this list will be used to return the names
	 * of associated files
	 * @param outFileURLs this list will be used to return the URLs
	 * of associated files
	 * @return String object containing OWL description of environmental data
	 */
	public String requestEnvironmentFromURL(String url, ArrayList<String> outFilenames, ArrayList<String> outFileURLs) {

		//System.out.print("\ndownload environment '"+Util.getFilenameFromURL(url)+"': ");
		
		String env = null;
		if (url != null) {

			String responseBody = requestData(url);

			if (responseBody!=null && (responseBody.startsWith("[") || responseBody.startsWith("{"))) {

				try {
					// Create a list of returned environments and their time stamps, ...
					EnvironmentList envs = new EnvironmentList(responseBody);
					responseBody = envs.get(0);

					if (responseBody != null && !responseBody.isEmpty()) {
						env = responseBody;	

						if (outFilenames != null) {
							outFilenames.clear();
							String[] names = envs.getFilenames(0);
							if (names != null) {
								for (String name : names) {
									outFilenames.add(name);
								}								
							}				
						}

						if (outFileURLs  != null) {
							outFileURLs.clear();
							String[] ourls = envs.getFileURLs(0);
							if (ourls != null) {
								for (String ourl : ourls) {
									outFileURLs.add(ourl);
								}								
							}				

						}
					}

				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}

			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return env;
	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download robot descriptions from the DB
	//
	
	
	@Override
	public String requestRobot(String uid) {
		if (uid != null) {
			return requestRobotFromURL(roboEarthBaseURL+requestRobotSuffix+uid);
		} else {
			System.out.println("\ndownload robot '"+uid+"': parameter error (null pointer)");
		}
		return null;
	}

	/**
	 * Requests a robot description.
	 * 
	 * @param url the URL of the requested robot description
	 * @return String object containing SRDL description of a robot
	 */
	public String requestRobotFromURL(String url) {

		//System.out.print("\ndownload robot '"+Util.getFilenameFromURL(url)+"': ");
		
		String responseBody = "";
		if (url != null) {

			responseBody = requestData(url);
			if (responseBody!= null && (responseBody.startsWith("[") || responseBody.startsWith("{"))) {

				try {
					// Create a list of returned robots and their time stamps, ...
					RobotList robots = new RobotList(responseBody);
					responseBody = robots.get(0);

				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}

			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return responseBody;
	}
	
	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download object models from the DB
	//
	
	/**
	 * Requests information about an object.
	 * 
	 * @param uid the object's unique identifier 
	 * @return String object containing OWL description of the object
	 */
	public String requestObject(String uid) {
		return requestObject(uid, null, null);
	}
	
	@Override
	public String requestObject(String uid, ArrayList<String> outFilenames, ArrayList<String> outFileURLs) {
		if (uid != null) {
			return requestObjectFromURL(roboEarthBaseURL+requestObjectSuffix+uid, outFilenames, outFileURLs);
		} else {
			System.out.println("\ndownload object '"+uid+"': parameter error (null pointer)");
			return null;	
		}
		
	}

	/**
	 * Requests information about an object.
	 * 
	 * @param url the object's URL
	 * @return String object containing OWL description of the object
	 */
	public String requestObjectFromURL(String url) {
		return requestObjectFromURL(url, null, null);
	}
	
	/**
	 * Requests information about an object. The names and URLs of binary 
	 * files related to the object represented by its own URL will be put 
	 * into the given lists. Any element that might be in the lists before 
	 * the method call will be removed first. Both list parameters may be 
	 * null. In this case only the OWL encoded description is requested.
	 * 
	 * @param url the object's URL
	 * @param outFilenames this list will be used to return the names
	 * of associated files
	 * @param outFileURLs this list will be used to return the URLs
	 * of associated files
	 * @return String object containing OWL description of the object
	 */
	public String requestObjectFromURL(String url, ArrayList<String> outFilenames, ArrayList<String> outFileURLs) {

		//System.out.print("\ndownload object '"+Util.getFilenameFromURL(url)+"': ");
		
		String obj = null;

		if (url != null) {

			String responseBody = requestData(url);
			if (responseBody!=null && (responseBody.startsWith("[") || responseBody.startsWith("{"))) {

				try {
					// Create a list of returned object models and their time stamps, ...
					ObjectList objects = new ObjectList(responseBody);
					responseBody = objects.get(0);

					if (responseBody != null && !responseBody.isEmpty()) {
						obj = responseBody;
						
						if (outFilenames != null) {
							outFilenames.clear();
							String[] names = objects.getFilenames(0);
							if (names != null) {
								for (String name : names) {
									outFilenames.add(name);
								}								
							}				
						}

						if (outFileURLs  != null) {
							outFileURLs.clear();
							String[] ourls = objects.getFileURLs(0);
							if (ourls != null) {
								for (String ourl : ourls) {
									outFileURLs.add(ourl);
								}								
							}				
						}							

					}

				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}

			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return obj;

	}

	
	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Download binary files
	//
	
	/**
	 * Requests a binary file.
	 * 
	 * @param fileURL the URL of the requested file
	 * @return the content of the requested file in a byte array
	 */
	public byte[] requestBinaryFile(URL fileURL) {

//		System.out.print("\ndownload file '"+fileURL+"': ");
		
		byte[] requestedFile = null;

		if (fileURL != null) {

			InputStream is = null;
			
			try {

				URLConnection urlCon = fileURL.openConnection();
				urlCon.setDoInput(true);
				urlCon.setConnectTimeout(timoutInMs);
				urlCon.setReadTimeout(timoutInMs);
				urlCon.connect();
				is = urlCon.getInputStream();
				
				ByteArrayOutputStream bos = new ByteArrayOutputStream(10240);	
				byte[] data = new byte[4096];				
				int i = 0;
				while ((i = is.read(data)) != -1) {
					bos.write(data, 0, i);
				}
				is.close();

				requestedFile = bos.toByteArray();
//				System.out.println("granted ["+(requestedFile.length/1024)+"KiB]");

			} catch (Exception e) {
				System.out.print("rejected, ");
				System.out.println(e.getLocalizedMessage());
				
				if (is != null) {
					try {
						is.close();
					} catch (IOException ioe) {}
				}
			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return requestedFile;
		
	}
	
	/**
	 * Requests a binary file.
	 * 
	 * @param fileURL the URL of the requested file
	 * @param targetPath the local path to store the file
	 * @return the requested file or null, if the file couldn't be found
	 */
	public File requestBinaryFile(URL fileURL, String targetPath) {

//		System.out.print("\ndownload file '"+fileURL+"' to '"+targetPath+"'): ");
		
		File requestedFile = null;

		if (fileURL != null && targetPath != null) {

			InputStream is = null;
			FileOutputStream fos = null;
			
			try {
				
				URLConnection urlCon = fileURL.openConnection();
				urlCon.setDoInput(true);
				urlCon.setConnectTimeout(timoutInMs);
				urlCon.setReadTimeout(timoutInMs);
				urlCon.connect();
				is = urlCon.getInputStream();
				
				if (!targetPath.endsWith(File.separator)) {
					targetPath += File.separator;
				}
				File fileToWrite = new File(targetPath+Util.getFilenameFromURL(fileURL.toString()));
				if (fileToWrite.exists()) {
					fileToWrite.delete();
				}
				fileToWrite.createNewFile();
				fos = new FileOutputStream(fileToWrite);	
				
				byte[] data = new byte[4096];
				int i = 0;
				while ((i = is.read(data)) != -1) {
					fos.write(data, 0, i);
				}
				fos.close();
				is.close();
				
				requestedFile = fileToWrite;
//				System.out.println("granted ["+(requestedFile.length()/1024)+"KiB]");
				
			} catch (Exception e) {
				System.out.print("rejected, ");
				System.out.println(e.getLocalizedMessage());
				
				if (is != null) {
					try {
						is.close();
					} catch (IOException ioe) {}
				}
				if (fos != null) {
					try {
						fos.close();
					} catch (IOException ioe) {}
				}
			}

		} else {
			System.out.println("parameter error (null pointer)");
		}

		return requestedFile;

	}
	
	@Override
	public byte[] requestObjectBinaryFile(String objectUID, String filename) {

		byte[] requestedFile = null;

		if (objectUID != null && filename != null) {

			String fileURL = roboEarthBaseURL + requestObjectBinariesSuffix
					+ objectUID.replace(".", "/") + "/" + filename;
			
			try {
				URL url = new URL(fileURL);
				requestedFile = requestBinaryFile(url);
			} catch (MalformedURLException e) {
				System.out.print("\ndownload file '" + fileURL + "' "
						+ "(obj. uid '" + objectUID + "'): ");
				System.out.println(e.getLocalizedMessage());
			}

		} else {
			System.out.println("\ndownload file '"+filename+"' "
					+"(obj. uid '" + objectUID + "'): parameter error (null pointer)");
		}
		return requestedFile;

	}
	
	@Override
	public File requestObjectBinaryFile(String objectUID, String filename, String targetPath) {

		File file = null;

		if (objectUID != null && filename != null && targetPath != null) {
			
			String fileURL = roboEarthBaseURL + requestObjectBinariesSuffix
					+ objectUID.replace(".", "/") + "/" + filename;

			try {
				URL url = new URL(fileURL);
				file = requestBinaryFile(url, targetPath);
			} catch (MalformedURLException e) {
				System.out.print("\ndownload file '" + fileURL + "' "
						+ "(obj. uid '" + objectUID + "'): ");
				System.out.println(e.getLocalizedMessage());
			}			
			
		} else {
			System.out.println("\ndownload file '"+filename+"' "
					+ "(obj. uid '" + objectUID + "'): parameter error (null pointer)");
		}
		return file;

	}
	
	@Override
	public byte[] requestEnvironmentBinaryFile(String envUID, String filename) {

		byte[] requestedFile = null;

		if (envUID != null && filename != null) {

			String fileURL = roboEarthBaseURL + requestEnvironmentBinariesSuffix
					+ envUID.replace(".", "/") + "/" + filename;
			
			try {
				URL url = new URL(fileURL);
				requestedFile = requestBinaryFile(url);
			} catch (MalformedURLException e) {
				System.out.print("\ndownload file '" + fileURL + "' "
						+ "(env. uid '" + envUID + "'): ");
				System.out.println(e.getLocalizedMessage());
			}

		} else {
			System.out.println("\ndownload file '" + filename + "' "
					+ "(env. uid '" + envUID + "'): parameter error (null pointer)");
		}
		return requestedFile;

	}
	
	@Override
	public File requestEnvironmentBinaryFile(String envUID, String filename, String targetPath) {

		File file = null;

		if (envUID != null && filename != null && targetPath != null) {
			
			String fileURL = roboEarthBaseURL + requestEnvironmentBinariesSuffix
					+ envUID.replace(".", "/") + "/" + filename;

			try {
				URL url = new URL(fileURL);
				file = requestBinaryFile(url, targetPath);
			} catch (MalformedURLException e) {
				System.out.print("\ndownload file '" + fileURL + "' "
						+ "(env. uid '" + envUID + "'): ");
				System.out.println(e.getLocalizedMessage());
			}			
			
		} else {
			System.out.println("\ndownload file '" + filename + "' "
					+ "(env. uid '" + envUID + "'): parameter error (null pointer)");
		}
		return file;

	}


	/**
	 * Requests data from the given URL via a HTTP Get request.
	 * 
	 * @param requestURL URL to request from
	 * @return JSON encoded data
	 */
	protected static String requestData(String requestURL) {

		String responseBody = null;

		if (requestURL != null) {

			HttpClient httpclient = null;
			try {

				httpclient = new DefaultHttpClient();
				HttpParams params = httpclient.getParams();
				HttpConnectionParams.setConnectionTimeout(params, timoutInMs);
				HttpConnectionParams.setSoTimeout(params, timoutInMs);
				
				HttpGet httpget = new HttpGet(requestURL);
				HttpResponse response = httpclient.execute(httpget);
				if (isHTTPRequestProcessedCorrectly(response)) {
					HttpEntity entitity = response.getEntity();
					responseBody = EntityUtils.toString(entitity);		
				}

			} catch (Exception e) {
				System.out.println(e.getLocalizedMessage());
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}

		}

		return responseBody;

	}

	
	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Upload information to the DB
	//
	
	
	@Override
	public boolean submitActionRecipe(OWLOntology actionRecipe, String cls, String id, String description) {

		System.out.print("\nupload recipe '"+cls+"."+id+"': ");
		
		boolean ok = false;

		if (apiKey != null && actionRecipe != null && cls != null && 
				id != null && description != null) {

			String owlData = OWLIO.saveOntologyToString(actionRecipe);
			JsonRootNode json = aJsonObject(
					aJsonField("class", aJsonString(cls)),
					aJsonField("id", aJsonString(id)),
					aJsonField("description", aJsonString(description)),
					aJsonField("recipe", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			ok = submitData(JSON_FORMATTER.format(json), roboEarthBaseURL+submitActionRecipeSuffix);

		} else {
			System.out.println("parameter error (null pointer)");
		}

		return ok;

	}

	@Override
	public boolean submitEnvironment(OWLOntology map, String cls, String id, String description) {

		System.out.print("\nupload environment '"+cls+"."+id+"': ");
		
		boolean ok = false;

		if (apiKey != null && map != null && cls != null && 
				id != null && description != null) {

			String owlData = OWLIO.saveOntologyToString(map);
			JsonRootNode json = aJsonObject(
					aJsonField("class", aJsonString(cls)),
					aJsonField("id", aJsonString(id)),
					aJsonField("description", aJsonString(description)),
					aJsonField("environment", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			ok = submitData(JSON_FORMATTER.format(json), roboEarthBaseURL+submitEnvironmentSuffix);

		} else {
			System.out.println("parameter error (null pointer)");
		}

		return ok;

	}

	@Override
	public boolean submitEnvironment(OWLOntology map, String cls, String id, 
			String description, ArrayList<File> binaryFiles) {

		boolean ok = submitEnvironment(map, cls, id, description);

		if (ok && binaryFiles != null) {
			String uid = cls+"."+id;
			for (File f : binaryFiles) {
				if (ok) {
					ok &= submitEnvironmentBinaryFile(uid, f);	
				}
			}
			if (!ok) {
				deleteEnvironment(uid);
			}
		}

		return ok;

	}

	@Override
	public boolean submitEnvironment(OWLOntology map, String cls, String id, 
			String description, ArrayList<byte[]> binaryData, 
			ArrayList<String> filenames) {

		boolean ok = submitEnvironment(map, cls, id, description);

		if (ok && binaryData != null && filenames != null &&
				binaryData.size() == filenames.size()) {
			String uid = cls+"."+id;
			for (int i=0;i< binaryData.size() && ok; i++) {
				ok &= submitEnvironmentBinaryFile(uid, binaryData.get(i), filenames.get(i));
			}
			if (!ok) {
				deleteEnvironment(uid);
			}
		}

		return ok;

	}

	@Override
	public boolean submitObject(OWLOntology object, String cls, String id, String description) {

		System.out.print("\nupload object '"+cls+"."+id+"': ");
		
		boolean ok = false;

		if (apiKey != null && object != null && cls != null && id != null && description != null) {

			String owlData = OWLIO.saveOntologyToString(object);
			JsonRootNode json = aJsonObject(
					aJsonField("class", aJsonString(cls)),
					aJsonField("id", aJsonString(id)),
					aJsonField("description", aJsonString(description)),
					aJsonField("object_description", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			ok = submitData(JSON_FORMATTER.format(json), roboEarthBaseURL+submitObjectSuffix);

		} else {
			System.out.println("parameter error (null pointer)");
		}

		return ok;

	}

	@Override
	public boolean submitObject(OWLOntology object, String cls, String id, 
			String description, ArrayList<File> binaryFiles) {

		boolean ok = submitObject(object, cls, id, description);

		if (ok && binaryFiles != null) {
			String uid = cls+"."+id;
			for (File f : binaryFiles) {
				if (ok) {
					ok &= submitObjectBinaryFile(uid, f);	
				}
			}
			if (!ok) {
				deleteObject(uid);
			}
		}

		return ok;

	}

	@Override
	public boolean submitObject(OWLOntology object, String cls, String id, 
			String description, ArrayList<byte[]> binaryData, 
			ArrayList<String> filenames) {

		boolean ok = submitObject(object, cls, id, description);

		if (ok && binaryData != null && filenames != null &&
				binaryData.size() == filenames.size()) {
			String uid = cls+"."+id;
			for (int i=0;i< binaryData.size() && ok; i++) {
				ok &= submitObjectBinaryFile(uid, binaryData.get(i), filenames.get(i));
			}
			if (!ok) {
				deleteObject(uid);
			}
		}

		return ok;

	}

	@Override
	public boolean submitObjectBinaryFile(String uid, final File file) {

		System.out.print("\nupload object file '"+file.getName()+"' (uid '"+uid+"')");
		
		FileInputStream fis;
		try {
			fis = new FileInputStream(file);
		} catch (Exception e) {
			System.out.println(": Couldn't create file input stream!");
			return false;
		}

		if (!file.canRead()) {
			System.out.println(": Can't read from file!");
			return false;
		}
		
		System.out.print(" ["+(file.length()/1024)+"KiB]: ");

		InputStreamBody streamBody = new InputStreamBody(fis, "application/octet-stream", file.getName()) {
			@Override
			public long getContentLength() {
				return file.length();
			}
		};		

		return submitBinaryFile(roboEarthBaseURL+submitObjectBinariesSuffix+uid, streamBody);

	}

	@Override
	public boolean submitObjectBinaryFile(String uid, final byte[] data, String filename) {

		System.out.print("\nupload object file '"+filename+"' (uid '"+uid+"')");
		
		if (data == null || filename == null || filename.length() == 0) {
			System.out.println(": parameter error (null pointer)");
			return false;
		}
		
		System.out.print(" ["+(data.length/1024)+"KiB]: ");

		ByteArrayInputStream bis = new ByteArrayInputStream(data);

		InputStreamBody streamBody = new InputStreamBody(bis, "application/octet-stream", filename) {
			@Override
			public long getContentLength() {
				return data.length;
			}
		};

		return submitBinaryFile(roboEarthBaseURL+submitObjectBinariesSuffix+uid, streamBody);

	}
	
	@Override
	public boolean submitEnvironmentBinaryFile(String uid, final File file) {

		System.out.print("\nupload environment file '"+file.getName()+"' (uid '"+uid+"')");
		
		FileInputStream fis;
		try {
			fis = new FileInputStream(file);
		} catch (Exception e) {
			System.out.println(": Couldn't create file input stream!");
			return false;
		}

		if (!file.canRead()) {
			System.out.println(": Can't read from file!");
			return false;
		}
		
		System.out.print(" ["+(file.length()/1024)+"KiB]: ");

		InputStreamBody streamBody = new InputStreamBody(fis, "application/octet-stream", file.getName()) {
			@Override
			public long getContentLength() {
				return file.length();
			}
		};		

		return submitBinaryFile(roboEarthBaseURL+submitEnvironmentBinariesSuffix+uid, streamBody);

	}
	
	@Override
	public boolean submitEnvironmentBinaryFile(String uid, final byte[] data, String filename) {

		System.out.print("\nupload environment file '"+filename+"' (uid '"+uid+"')");
		
		if (data == null || filename == null || filename.length() == 0) {
			System.out.println(": parameter error (null pointer)");
			return false;
		}

		System.out.print(" ["+(data.length/1024)+"KiB]: ");
		
		ByteArrayInputStream bis = new ByteArrayInputStream(data);

		InputStreamBody streamBody = new InputStreamBody(bis, "application/octet-stream", filename) {
			@Override
			public long getContentLength() {
				return data.length;
			}
		};

		return submitBinaryFile(roboEarthBaseURL+submitEnvironmentBinariesSuffix+uid, streamBody);

	}
	
	/**
	 * Submits a binary file (related to the resource specified by the given
	 * URL) to the RoboEarth DB.
	 * 
	 * @param url the URL of an DB entry, the binary file is related to
	 * @param isb the InputStreamBody containing the binary file
	 * @return <tt>true</tt> - if submission was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	protected boolean submitBinaryFile(String url, InputStreamBody isb) {

		boolean ok = false;

		if (apiKey != null && url != null && isb != null && 
				isb.getFilename() != null && isb.getFilename().length() > 0) {

			HttpClient httpclient = null;

			try {

				httpclient = new DefaultHttpClient();
				HttpParams params = httpclient.getParams();
				HttpConnectionParams.setConnectionTimeout(params, timoutInMs);
				HttpConnectionParams.setSoTimeout(params, timoutInMs);
				
				HttpPost httppost = new HttpPost(url);

				StringBody apiKeyBody = new StringBody(apiKey);
				StringBody typeBody = new StringBody(isb.getFilename());

				MultipartEntity mpEntity = new MultipartEntity();
				mpEntity.addPart("api_key", apiKeyBody);
				mpEntity.addPart("type", typeBody);
				mpEntity.addPart("file", isb);
				httppost.setEntity(mpEntity);

				HttpResponse response = httpclient.execute(httppost);
				ok = isHTTPRequestProcessedCorrectly(response);

			} catch (Exception e) {
				System.out.println(e.getLocalizedMessage());
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
				System.out.println();
			}

		} else {
			System.out.println("parameter error (null pointer)");
		}

		return ok;

	}


	/**
	 * Submits data to given URL via a HTTP Post request.
	 * 
	 * @param jsonData JSON encoded data
	 * @param submitURL URL of RoboEarth DB to submit to
	 * @return <tt>true</tt> - if submission was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	protected static boolean submitData(String jsonData, String submitURL) {

		boolean ok = false;

		if (jsonData != null && submitURL != null) {

			HttpClient httpclient = null;

			try {

				httpclient = new DefaultHttpClient();
				HttpParams params = httpclient.getParams();
				HttpConnectionParams.setConnectionTimeout(params, timoutInMs);
				HttpConnectionParams.setSoTimeout(params, timoutInMs);
				
				Header header = new BasicHeader("Content-Type","application/json");
				StringEntity entity = new StringEntity(jsonData, "UTF-8");

				HttpPost httppost = new HttpPost(submitURL);
				httppost.setEntity(entity);
				httppost.setHeader(header);

				HttpResponse response = httpclient.execute(httppost);
				ok = isHTTPRequestProcessedCorrectly(response);

			} catch (Exception e) {
				System.out.println(e.getLocalizedMessage());
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}

		}

		return ok;

	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Query Sesame repository
	//
	
	@Override
	public String queryActionRecipeDB(String seRQLquery) {

//		System.out.print("\nquery recipes: ");
		return querySemanticDB(seRQLquery, "recipes");

	}

	@Override
	public String queryObjectDB(String seRQLquery) {

//		System.out.print("\nquery objects: ");
		return querySemanticDB(seRQLquery, "objects");

	}

	@Override
	public String queryEnvironmentDB(String seRQLquery) {

//		System.out.print("\nquery environments: ");
		return querySemanticDB(seRQLquery, "environments");

	}

	/**
	 * Sends the given SeRQL query to the RoboEarth DB via a HTTP Post request.
	 * 
	 * @param seRQLquery a SeRQL query
	 * @param repository the name of the repository to query (must be 
	 * 'recipes', 'objects' or 'environments')
	 * @return result as a String
	 */
	protected static String querySemanticDB(String seRQLquery, String repository) {

		String result = null;

		if (seRQLquery != null) {

			HttpClient httpclient = null;

			try {

				httpclient = new DefaultHttpClient();
				HttpParams params = httpclient.getParams();
				HttpConnectionParams.setConnectionTimeout(params, timoutInMs);
				HttpConnectionParams.setSoTimeout(params, timoutInMs);

				JsonRootNode json = aJsonObject(
						aJsonField("query", aJsonString(seRQLquery)),
						aJsonField("repository", aJsonString(repository))
						);

				String jsonText = JSON_FORMATTER.format(json);

				Header header = new BasicHeader("Content-Type","application/json");
				StringEntity entity = new StringEntity(jsonText, "UTF-8");

				HttpPost httppost = new HttpPost(roboEarthBaseURL+querySuffix);
				httppost.setEntity(entity);
				httppost.setHeader(header);

				HttpResponse response = httpclient.execute(httppost);
				if (isHTTPRequestProcessedCorrectly(response)) {
					HttpEntity entitity = response.getEntity();
					if (entitity != null) {
						result = EntityUtils.toString(entitity);
					}
				}

			} catch (Exception e) {
				System.out.println(e.getLocalizedMessage());
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}

		}

		return result;

	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Delete information from the DB
	//
	
	@Override
	public boolean deleteActionRecipe(String uid) {

//		System.out.print("\ndelete recipe '"+uid+"': ");
		return deleteData(roboEarthBaseURL+requestActionRecipeSuffix+uid+"/"+apiKey);

	}

	@Override
	public boolean deleteObject(String uid) {

//		System.out.print("\ndelete object '"+uid+"': ");
		return deleteData(roboEarthBaseURL+requestObjectSuffix+uid+"/"+apiKey);

	}

	@Override
	public boolean deleteEnvironment(String uid) {

//		System.out.print("\ndelete environment '"+uid+"': ");
		return deleteData(roboEarthBaseURL+requestEnvironmentSuffix+uid+"/"+apiKey); 

	}

	@Override
	public boolean deleteObjectBinaryFile(String objectUid, String fileID) {

//		System.out.print("\ndelete file (id:'"+fileID+"', obj. uid:'"+objectUid+"'): ");
		return deleteData(roboEarthBaseURL+requestObjectSuffix+objectUid+"/"+apiKey+"/"+fileID);

	}

	@Override
	public boolean deleteEnvironmentBinaryFile(String envUid, String fileID) {

//		System.out.print("\ndelete file (id:'"+fileID+"', env. uid:'"+envUid+"'): ");
		return deleteData(roboEarthBaseURL+requestEnvironmentSuffix+envUid+"/"+apiKey+"/"+fileID);

	}
	
	/**
	 * Deletes an entry from the RoboEarth DB via a HTTP Delete request.
	 * 
	 * @param deleteURL URL of RoboEarth DB entry to delete
	 * @return <tt>true</tt> - if DB entry was successfully deleted, 
	 * <tt>false</tt> - otherwise
	 */
	protected static boolean deleteData(String deleteURL) {

		boolean ok = false;

		if (deleteURL != null) {

			HttpClient httpclient = null;

			try {

				httpclient = new DefaultHttpClient();
				HttpParams params = httpclient.getParams();
				HttpConnectionParams.setConnectionTimeout(params, timoutInMs);
				HttpConnectionParams.setSoTimeout(params, timoutInMs);
				
				HttpDelete httpdel = new HttpDelete(deleteURL);
				HttpResponse response = httpclient.execute(httpdel);
				ok = isHTTPRequestProcessedCorrectly(response);

			} catch (Exception e) {
				System.out.println(e.getLocalizedMessage());
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}
		}

		return ok;

	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Update information in the DB
	//
	
	@Override
	public boolean updateActionRecipe(String uid, OWLOntology actionRecipe, String description) {

//		System.out.print("\nupdate recipe '"+uid+"': ");
		
		boolean ok = false;

		if (apiKey != null && actionRecipe != null && uid != null && description != null) {

			String owlData = OWLIO.saveOntologyToString(actionRecipe);
			JsonRootNode json = aJsonObject(
					aJsonField("description", aJsonString(description)),
					aJsonField("recipe", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			ok = updateData(JSON_FORMATTER.format(json), roboEarthBaseURL+requestActionRecipeSuffix+uid);

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return ok;

	}

	@Override
	public boolean updateEnvironment(String uid, OWLOntology env, String description) {

//		System.out.print("\nupdate environment '"+uid+"': ");
		
		boolean ok = false;

		if (apiKey != null && env != null && uid != null && description != null) {

			String owlData = OWLIO.saveOntologyToString(env);
			JsonRootNode json = aJsonObject(
					aJsonField("description", aJsonString(description)),
					aJsonField("environment", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			ok = updateData(JSON_FORMATTER.format(json), roboEarthBaseURL+requestEnvironmentSuffix+uid);

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return ok;

	}
	

	public boolean updateEnvironment(String uid, String env, String description) {

//		System.out.print("\nupdate environment '"+uid+"': ");
		
		boolean ok = false;

		if (apiKey != null && env != null && uid != null && description != null) {

			String owlData = env;
			JsonRootNode json = aJsonObject(
					aJsonField("description", aJsonString(description)),
					aJsonField("environment", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			ok = updateData(JSON_FORMATTER.format(json), roboEarthBaseURL+requestEnvironmentSuffix+uid);

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return ok;

	}

	@Override
	public boolean updateEnvironmentBinaryFile(String uid, File file) {

		boolean ok = false;

		if (uid != null && file != null && file.canRead()) {
//			System.out.print("\nupdate file for uid '"+uid+"', (1/2: delete '"+file.getName()+"'): ");
			ok = deleteEnvironmentBinaryFile(uid, file.getName());
			if (ok) {
//				System.out.print("\nupdate file for uid '"+uid+"', (2/2: submit '"+file.getName()+"'): ");
				ok = submitEnvironmentBinaryFile(uid, file);	
			}
		} else {
//			System.out.println("\nupdate file for uid '"+uid+"': parameter error (null pointer)");
		}

		return ok;

	}

	@Override
	public boolean updateEnvironmentBinaryFile(String uid, byte[] data, String filename) {

		boolean ok = false;

		if (uid != null && data != null && filename != null) {
//			System.out.print("\nupdate file for uid '"+uid+"', (1/2: delete '"+filename+"'): ");
			ok = deleteEnvironmentBinaryFile(uid, filename);
			if (ok) {
//				System.out.print("\nupdate file for uid '"+uid+"', (2/2: submit '"+filename+"'): ");
				ok = submitEnvironmentBinaryFile(uid, data, filename);	
			}
		} else {
//			System.out.println("\nupdate file for uid '"+uid+"': parameter error (null pointer)");
		}

		return ok;

	}
	
	@Override
	public boolean updateObject(String uid, OWLOntology object, String description) {

//		System.out.print("\nupdate object descr. '"+uid+"': ");
		
		boolean ok = false;

		if (apiKey != null && object != null && uid != null && description != null) {

			String owlData = OWLIO.saveOntologyToString(object);
			JsonRootNode json = aJsonObject(
					aJsonField("description", aJsonString(description)),
					aJsonField("object_description", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			
			ok = updateData(JSON_FORMATTER.format(json), roboEarthBaseURL+requestObjectSuffix+uid);

			if(!ok)
				System.err.println("updateData failed");
			
		} else {
			System.out.println("parameter error (null pointer)");
		}
		return ok;

	}
	
	@Override
	public boolean updateObject(String uid, String object, String description) {

//		System.out.print("\nupdate object descr. '"+uid+"': ");
		
		boolean ok = false;

		if (apiKey != null && object != null && uid != null && description != null) {

			String owlData = object;
			JsonRootNode json = aJsonObject(
					aJsonField("description", aJsonString(description)),
					aJsonField("object_description", aJsonString(owlData)),
					aJsonField("api_key", aJsonString(apiKey))
					);
			
			ok = updateData(JSON_FORMATTER.format(json), roboEarthBaseURL+requestObjectSuffix+uid);

			if(!ok)
				System.err.println("updateData failed");
			
		} else {
			System.out.println("parameter error (null pointer)");
		}
		return ok;

	}

	@Override
	public boolean updateObjectBinaryFile(String uid, File file) {

		boolean ok = false;

		if (uid != null && file != null && file.canRead()) {
//			System.out.print("\nupdate file for uid '"+uid+"', (1/2: delete '"+file.getName()+"'): ");
			ok = deleteObjectBinaryFile(uid, file.getName());
			if (ok) {
//				System.out.print("\nupdate file for uid '"+uid+"', (2/2: submit '"+file.getName()+"'): ");
				ok = submitObjectBinaryFile(uid, file);	
			}
		} else {
//			System.out.println("\nupdate file for uid '"+uid+"': parameter error (null pointer)");
		}

		return ok;

	}

	@Override
	public boolean updateObjectBinaryFile(String uid, byte[] data, String filename) {

		boolean ok = false;

		if (uid != null && data != null && filename != null) {
//			System.out.print("\nupdate file for uid '"+uid+"', (1/2: delete '"+filename+"'): ");
			ok = deleteObjectBinaryFile(uid, filename);
			if (ok) {
//				System.out.print("\nupdate file for uid '"+uid+"', (2/2: submit '"+filename+"'): ");
				ok = submitObjectBinaryFile(uid, data, filename);	
			}
		} else {
//			System.out.println("\nupdate file for uid '"+uid+"': parameter error (null pointer)");
		}

		return ok;

	}

	/**
	 * Updates an entry of the RoboEarth DB with the given data via a HTTP Put request.
	 * 
	 * @param jsonData JSON encoded data
	 * @param updateURL URL of RoboEarth DB entry to update
	 * @return <tt>true</tt> - if the DB entry was successfully updated<br>
	 * <tt>false</tt> - otherwise
	 */
	protected static boolean updateData(String jsonData, String updateURL) {

		boolean ok = false;

		if (jsonData != null && updateURL != null) {

			HttpClient httpclient = null;

			try {

				httpclient = new DefaultHttpClient();
				HttpParams params = httpclient.getParams();
				HttpConnectionParams.setConnectionTimeout(params, timoutInMs);
				HttpConnectionParams.setSoTimeout(params, timoutInMs);
				
				Header header = new BasicHeader("Content-Type","application/json");
				StringEntity entity = new StringEntity(jsonData, "UTF-8");

				HttpPut httpput = new HttpPut(updateURL);
				httpput.setEntity(entity);
				httpput.setHeader(header);
				
				HttpResponse response = httpclient.execute(httpput);
				ok = isHTTPRequestProcessedCorrectly(response);
				System.err.println(response.toString());
			} catch (Exception e) {
				System.out.println(e.getLocalizedMessage());
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}

		}

		return ok;

	}

	
	// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 
	// 
	// Search for information in the DB
	//
	
	@Override
	public String[] searchActionRecipes(String searchID, ArrayList<String> outUIDs) {

//		System.out.print("\nsearch recipes '"+searchID+"': ");
		
		String[] actionRecipes = null;
		if (searchID != null) {

			String responseBody = requestData(roboEarthBaseURL+requestActionRecipeSuffix+searchID);
			if (responseBody != null && (responseBody.startsWith("[") || responseBody.startsWith("{"))) {

				try {
					// Create a list of returned recipes and their time stamps, ...
					RecipeList recipes = new RecipeList(responseBody);

					if (recipes != null) {

						if (outUIDs != null) {
							outUIDs.clear();
						}

						actionRecipes = new String[recipes.size()];
						for (int i=0;i<actionRecipes.length; i++) {
							responseBody = recipes.get(i);
							actionRecipes[i] = responseBody;
							if (outUIDs != null) {
								outUIDs.add(recipes.getUID(i));	
							}
						}
					}

				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}

			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return actionRecipes;

	}

	@Override
	public String[] searchEnvironments(String searchID, ArrayList<String> outUIDs,
			ArrayList<ArrayList<String>> outFilenames,
			ArrayList<ArrayList<String>> outFileURLs) {

//		System.out.print("\nsearch environments '"+searchID+"': ");
		
		String[] environments = null;
		if (searchID != null) {

			String responseBody = requestData(roboEarthBaseURL+requestEnvironmentSuffix+searchID);
			if (responseBody != null && (responseBody.startsWith("[") || responseBody.startsWith("{"))) {

				try {
					// Create a list of returned environments and their time stamps, ...
					EnvironmentList envs = new EnvironmentList(responseBody);

					if (envs != null) {

						if (outUIDs != null) {
							outUIDs.clear();
						}
						if (outFilenames != null) {
							outFilenames.clear();
						}
						if (outFileURLs != null) {
							outFileURLs.clear();
						}

						environments = new String[envs.size()];
						for (int i=0;i<environments.length; i++) {
							responseBody = envs.get(i);
							environments[i] = responseBody;
							if (outUIDs != null) {
								outUIDs.add(envs.getUID(i));	
							}
							if (outFilenames != null) {
								String[] names = envs.getFilenames(i);
								if (names != null) {
									ArrayList<String> nameList = new ArrayList<String>();
									for (String name : names) {
										nameList.add(name);
									}
									outFilenames.add(nameList);
								}
							}
							if (outFileURLs != null) {
								String[] urls = envs.getFileURLs(i);
								if (urls != null) {
									ArrayList<String> urlList = new ArrayList<String>();
									for (String url : urls) {
										urlList.add(url);
									}
									outFileURLs.add(urlList);
								}
							}
						}
					}

				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}

			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return environments;

	}

	@Override
	public String[] searchObjects(String searchID, ArrayList<String> outUIDs,
			ArrayList<ArrayList<String>> outFilenames,
			ArrayList<ArrayList<String>> outFileURLs) {

//		System.out.print("\nsearch objects '"+searchID+"': ");
		
		String[] objects = null;
		if (searchID != null) {

			String responseBody = requestData(roboEarthBaseURL+requestObjectSuffix+searchID);
			if (responseBody != null && (responseBody.startsWith("[") || responseBody.startsWith("{"))) {

				try {
					// Create a list of returned objects and their time stamps, ...
					ObjectList objList = new ObjectList(responseBody);

					if (objList != null) {

						if (outUIDs != null) {
							outUIDs.clear();
						}
						if (outFilenames != null) {
							outFilenames.clear();
						}
						if (outFileURLs != null) {
							outFileURLs.clear();
						}

						objects = new String[objList.size()];
						for (int i=0;i<objects.length; i++) {
							responseBody = objList.get(i);
							objects[i] = responseBody;
							if (outUIDs != null) {
								outUIDs.add(objList.getUID(i));	
							}
							if (outFilenames != null) {
								String[] names = objList.getFilenames(i);
								if (names != null) {
									ArrayList<String> nameList = new ArrayList<String>();
									for (String name : names) {
										nameList.add(name);
									}
									outFilenames.add(nameList);
								}
							}
							if (outFileURLs != null) {
								String[] urls = objList.getFileURLs(i);
								if (urls != null) {
									ArrayList<String> urlList = new ArrayList<String>();
									for (String url : urls) {
										urlList.add(url);
									}
									outFileURLs.add(urlList);
								}
							}
						}
					}

				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}

			}

		} else {
			System.out.println("parameter error (null pointer)");
		}
		return objects;

	}

	@Override
	public ArrayList<byte[]> request2dMap(String envUid, OWLOntology srdl,
			String baseScannerLink, String simpleMapNameWithoutExt) {

		System.out.println("\nrequest 2d map for env. '" + envUid + "': ");
		
		ArrayList<byte[]> map = null;
		
		if (envUid != null && srdl != null && baseScannerLink != null &&
				simpleMapNameWithoutExt != null) {

			double z = Inference.getZCoordinate(srdl, baseScannerLink);
			if (!Double.isNaN(z)) {
				
				JsonRootNode json = aJsonObject(
						aJsonField("env", aJsonString(envUid)),
						aJsonField("z_min", aJsonString(String.valueOf(z)))
						);
				
				String url = roboEarthBaseURL+mapExtractorSuffix;
				String jsonData = JSON_FORMATTER.format(json);
				String responseBody = requestService(jsonData, url);
				
				try {
					EnvironmentList envs = new EnvironmentList(responseBody);
					map = envs.get2dMap(0, simpleMapNameWithoutExt);
				} catch (InvalidSyntaxException e) {
					e.printStackTrace();
				}
				
			} else {
				System.out.println("couldn't determine pose of link '"+ baseScannerLink +"'");	
			}
			
		} else {
			System.out.println("parameter error (null pointer)");
		}
		
		return map;
		
	}
	
	@Override
	public ArrayList<byte[]> requestProjected2dMap(String envUid, double minZ,
			double maxZ, String simpleMapNameWithoutExt) {

		double z1, z2;
		
		if (minZ <= maxZ) {
			z1 = minZ;
			z2 = maxZ;
		} else {
			z2 = minZ;
			z1 = maxZ;			
		}
		
		System.out.print("\nrequest projected 2d map for env. '" + 
				envUid + "' (z1 "+z1+", z2 "+z2+"): ");
		
		ArrayList<byte[]> map = null;
		
		JsonRootNode json = aJsonObject(
				aJsonField("env", aJsonString(envUid)),
				aJsonField("z_min", aJsonString(String.valueOf(z1))),
				aJsonField("z_max", aJsonString(String.valueOf(z2)))
				);
		
		String url = roboEarthBaseURL+mapExtractorSuffix;
		String jsonData = JSON_FORMATTER.format(json);
		String responseBody = requestService(jsonData, url);
		
		try {
			EnvironmentList envs = new EnvironmentList(responseBody);
			map = envs.get2dMap(0, simpleMapNameWithoutExt);
		} catch (InvalidSyntaxException e) {
			e.printStackTrace();
		}
		
		return map;
		
	}
	
	protected static String requestService(String jsonData, String url) {

		String responseBody = null;

		if (jsonData != null && url != null) {
			
			HttpClient httpclient = null;

			try {

				httpclient = new DefaultHttpClient();
				HttpParams params = httpclient.getParams();
				HttpConnectionParams.setConnectionTimeout(params, timoutInMs);
				HttpConnectionParams.setSoTimeout(params, timoutInMs);
				
				Header header = new BasicHeader("Content-Type","application/json");
				StringEntity entity = new StringEntity(jsonData, "UTF-8");

				HttpPost httppost = new HttpPost(url);
				httppost.setEntity(entity);
				httppost.setHeader(header);

				HttpResponse response = httpclient.execute(httppost);
				if (isHTTPRequestProcessedCorrectly(response)) {
					HttpEntity entitity = response.getEntity();
					responseBody = EntityUtils.toString(entitity);
				}

			} catch (Exception e) {
				System.out.println(e.getLocalizedMessage());
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}
			
		}
		
		return responseBody;
		
	}
	
	/**
	 * Examines a HTTP response and analyzes whether the preceding request was processed successfully. 
	 * @param response a HTTP response
	 * @return <tt>true</tt> - if HTTP response's status code is greater or equal than 200 and lower than 300<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean isHTTPRequestProcessedCorrectly(HttpResponse response) {

		boolean ok;

		StatusLine stl = response.getStatusLine();
		int status = stl.getStatusCode();
		if (status >=200 && status <300) {
//			System.out.println("granted, "+status+" "+stl.getReasonPhrase());
			ok = true;
		} else {
//			System.out.println("rejected, "+status+" "+stl.getReasonPhrase());
			ok = false;
		}

		return ok;

	}

	/**
	 * 
	 * A RecipeList object contains a JSON parser, that tries to extract the <tt>recipe</tt>
	 * and <tt>timestamp</tt> fields from elements of an array named <tt>recipes</tt> of a 
	 * given JSON encoded string and presents the collected data in a convenient Java 
	 * AbstractList derived object. 
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	public static class RecipeList extends AbstractList<String> {

		private final JsonRootNode json;

		// pre-built JSON node selectors, used to extract specific nodes
		private final static JsonNodeSelector<JsonNode, List<JsonNode>> JSON_RECIPES = 
				JsonNodeSelectors.anArrayNode("recipes");

		private final static JsonNodeSelector<JsonNode, String> JSON_RECIPE = 
				JsonNodeSelectors.aStringNode("recipe");

		private final static JsonNodeSelector<JsonNode, String> JSON_TIMESTAMP = 
				JsonNodeSelectors.aNumberNode("timestamp");

		private final static JsonNodeSelector<JsonNode, String> JSON_AUTHOR = 
				JsonNodeSelectors.aStringNode("author");

		private final static JsonNodeSelector<JsonNode, String> JSON_DESCRIPTION = 
				JsonNodeSelectors.aStringNode("description");

		private final static JsonNodeSelector<JsonNode, String> JSON_UID = 
				JsonNodeSelectors.aStringNode("id");

		public RecipeList(String jsonText) throws InvalidSyntaxException {
			json = json_parser.parse(jsonText);
		}

		@Override
		public String get(int index) {

			JsonNode recipe = json.getArrayNode().get(index);
			List<JsonNode> recipeVariants = JSON_RECIPES.getValue(recipe);
			return JSON_RECIPE.getValue(recipeVariants.get(0));

		}

		public long getTimestamp(int index) {

			List<JsonNode> recipes = json.getArrayNode();
			List<JsonNode> recipeVariants = JSON_RECIPES.getValue(recipes.get(index));
			return Long.parseLong(JSON_TIMESTAMP.getValue(recipeVariants.get(0)));

		}

		public String getDescription(int index) {
			return JSON_DESCRIPTION.getValue(json.getArrayNode().get(index));
		}

		public String getAuthor(int index) {
			return JSON_AUTHOR.getValue(json.getArrayNode().get(index));
		}

		public String getUID(int index) {
			return JSON_UID.getValue(json.getArrayNode().get(index));
		}

		@Override
		public int size() {
			return json.getArrayNode().size();
		}

	}

	/**
	 * 
	 * A ObjectList object contains a JSON parser, that tries to extract the <tt>description</tt>
	 * and <tt>timestamp</tt> fields from elements of an array named <tt>object_description</tt>
	 * of a given JSON encoded string and presents the collected data in a convenient Java
	 * AbstractList derived object. 
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	public static class ObjectList extends AbstractList<String> {

		private final JsonRootNode json;

		// pre-built JSON node selectors, used to extract specific nodes
		private final static JsonNodeSelector<JsonNode, List<JsonNode>> JSON_OBJECTS = 
				JsonNodeSelectors.anArrayNode("object_description");

		private final static JsonNodeSelector<JsonNode, String> JSON_OBJECT = 
				JsonNodeSelectors.aStringNode("description");

		private final static JsonNodeSelector<JsonNode, String> JSON_TIMESTAMP = 
				JsonNodeSelectors.aNumberNode("timestamp");

		private final static JsonNodeSelector<JsonNode, String> JSON_AUTHOR = 
				JsonNodeSelectors.aStringNode("author");

		private final static JsonNodeSelector<JsonNode, String> JSON_DESCRIPTION = 
				JsonNodeSelectors.aStringNode("description");

		private final static JsonNodeSelector<JsonNode, String> JSON_UID = 
				JsonNodeSelectors.aStringNode("id");

		private final static JsonNodeSelector<JsonNode, Map<JsonStringNode,JsonNode>> JSON_FILES = 
				JsonNodeSelectors.anObjectNode("files");

		private final static JsonNodeSelector<JsonNode, String> JSON_URL = 
				JsonNodeSelectors.aStringNode("url");

		
		@Deprecated
		private final static JsonNodeSelector<JsonNode, String> JSON_IMAGE_PATH = 
		JsonNodeSelectors.aStringNode("files",fileIdObjImage,"url");

		@Deprecated
		private final static JsonNodeSelector<JsonNode, String> JSON_MODEL_PATH = 
		JsonNodeSelectors.aStringNode("files",fileIdObjModel,"url");

		@Deprecated
		private final static JsonNodeSelector<JsonNode, String> JSON_ARTICULATION_PATH = 
		JsonNodeSelectors.aStringNode("files",fileIdObjArticModel,"url");


		public ObjectList(String jsonText) throws InvalidSyntaxException {
			json = json_parser.parse(jsonText);
		}

		@Override
		public String get(int index) {

			JsonNode obj = json.getArrayNode().get(index);
			List<JsonNode> objVariants = JSON_OBJECTS.getValue(obj);
			return JSON_OBJECT.getValue(objVariants.get(0));

		}

		public long getTimestamp(int index) {

			List<JsonNode> objs = json.getArrayNode();
			List<JsonNode> objVariants = JSON_OBJECTS.getValue(objs.get(index));
			return Long.parseLong(JSON_TIMESTAMP.getValue(objVariants.get(0)));

		}

		public String getDescription(int index) {
			return JSON_DESCRIPTION.getValue(json.getArrayNode().get(index));
		}

		public String getAuthor(int index) {
			return JSON_AUTHOR.getValue(json.getArrayNode().get(index));
		}

		public String getUID(int index) {
			return JSON_UID.getValue(json.getArrayNode().get(index));
		}

		public String[] getFilenames(int index) {

			String[] filenames = null;

			JsonNode objNode = json.getArrayNode().get(index);

			if (JSON_FILES.matches(objNode)) {
				Map<JsonStringNode,JsonNode> files = JSON_FILES.getValue(objNode);
				filenames = new String[files.size()];
				Set<JsonStringNode> keys = files.keySet();
				int i = 0;
				for (JsonStringNode key : keys) {
					JsonNode fileNode = files.get(key);
					if (JSON_URL.matches(fileNode)) {
						filenames[i++] = key.getText();
					}
				}
			}

			return filenames;

		}

		public String[] getFileURLs(int index) {

			String[] fileURLs = null;

			JsonNode objNode = json.getArrayNode().get(index);

			if (JSON_FILES.matches(objNode)) {
				Map<JsonStringNode,JsonNode> files = JSON_FILES.getValue(objNode);
				fileURLs = new String[files.size()];
				Set<JsonStringNode> keys = files.keySet();
				int i = 0;
				for (JsonStringNode key : keys) {
					JsonNode urlNode = files.get(key);
					if (JSON_URL.matches(urlNode)) {
						fileURLs[i++] = JSON_URL.getValue(urlNode);
					}
				}
			}

			return fileURLs;

		}

		@Deprecated
		public String getPathToImage(int index) {
			String path = null;

			JsonNode objNode = json.getArrayNode().get(index);

			if (JSON_IMAGE_PATH.matches(objNode)) {
				path = JSON_IMAGE_PATH.getValue(objNode);
			}
			return path;
		}

		@Deprecated
		public String getPathToModel(int index) {
			String path = null;

			JsonNode objNode = json.getArrayNode().get(index);

			if (JSON_MODEL_PATH.matches(objNode)) {
				path = JSON_MODEL_PATH.getValue(objNode);
			}
			return path;
		}

		@Deprecated
		public String getPathToArticulationModel(int index) {
			String path = null;

			JsonNode objNode = json.getArrayNode().get(index);

			if (JSON_ARTICULATION_PATH.matches(objNode)) {
				path = JSON_ARTICULATION_PATH.getValue(objNode);
			}
			return path;
		}

		@Override
		public int size() {
			return json.getArrayNode().size();
		}

	}

	/**
	 * 
	 * A EnvironmentList object contains a JSON parser, that tries to extract the <tt>environment</tt>
	 * and <tt>timestamp</tt> fields from elements of an array named <tt>environments</tt>
	 * of a given JSON encoded string and presents the collected data in a convenient Java
	 * AbstractList derived object. 
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	public static class EnvironmentList extends AbstractList<String> {

		private final JsonRootNode json;

		// pre-built JSON node selectors, used to extract specific nodes
		private final static JsonNodeSelector<JsonNode, List<JsonNode>> JSON_ENVIRONMENTS = 
				JsonNodeSelectors.anArrayNode("environments");

		private final static JsonNodeSelector<JsonNode, String> JSON_ENVIRONMENT = 
				JsonNodeSelectors.aStringNode("environment");

		private final static JsonNodeSelector<JsonNode, String> JSON_TIMESTAMP = 
				JsonNodeSelectors.aNumberNode("timestamp");

		private final static JsonNodeSelector<JsonNode, String> JSON_AUTHOR = 
				JsonNodeSelectors.aStringNode("author");

		private final static JsonNodeSelector<JsonNode, String> JSON_DESCRIPTION = 
				JsonNodeSelectors.aStringNode("description");

		private final static JsonNodeSelector<JsonNode, String> JSON_UID = 
				JsonNodeSelectors.aStringNode("id");

		private final static JsonNodeSelector<JsonNode, Map<JsonStringNode,JsonNode>> JSON_FILES = 
			JsonNodeSelectors.anObjectNode("files");

		private final static JsonNodeSelector<JsonNode, String> JSON_URL = 
			JsonNodeSelectors.aStringNode("url");
		
		private final static JsonNodeSelector<JsonNode, Map<JsonStringNode,JsonNode>> JSON_MAPS = 
			JsonNodeSelectors.anObjectNode("2dmap");
		
		public EnvironmentList(String jsonText) throws InvalidSyntaxException {
			json = json_parser.parse(jsonText);
		}

		@Override
		public String get(int index) {

			JsonNode env = json.getArrayNode().get(index);
			List<JsonNode> envVariants = JSON_ENVIRONMENTS.getValue(env);
			return JSON_ENVIRONMENT.getValue(envVariants.get(0));

		}

		public long getTimestamp(int index) {

			List<JsonNode> env = json.getArrayNode();
			List<JsonNode> envVariants = JSON_ENVIRONMENTS.getValue(env.get(index));
			return Long.parseLong(JSON_TIMESTAMP.getValue(envVariants.get(0)));

		}

		public String getDescription(int index) {
			return JSON_DESCRIPTION.getValue(json.getArrayNode().get(index));
		}

		public String getAuthor(int index) {
			return JSON_AUTHOR.getValue(json.getArrayNode().get(index));
		}

		public String getUID(int index) {
			return JSON_UID.getValue(json.getArrayNode().get(index));
		}

		public String[] getFilenames(int index) {

			String[] filenames = null;

			JsonNode objNode = json.getArrayNode().get(index);

			if (JSON_FILES.matches(objNode)) {
				Map<JsonStringNode,JsonNode> files = JSON_FILES.getValue(objNode);
				filenames = new String[files.size()];
				Set<JsonStringNode> keys = files.keySet();
				int i = 0;
				for (JsonStringNode key : keys) {
					JsonNode fileNode = files.get(key);
					if (JSON_URL.matches(fileNode)) {
						filenames[i++] = key.getText();
					}
				}
			}

			return filenames;

		}

		public String[] getFileURLs(int index) {

			String[] fileURLs = null;

			JsonNode objNode = json.getArrayNode().get(index);

			if (JSON_FILES.matches(objNode)) {
				Map<JsonStringNode,JsonNode> files = JSON_FILES.getValue(objNode);
				fileURLs = new String[files.size()];
				Set<JsonStringNode> keys = files.keySet();
				int i = 0;
				for (JsonStringNode key : keys) {
					JsonNode urlNode = files.get(key);
					if (JSON_URL.matches(urlNode)) {
						fileURLs[i++] = JSON_URL.getValue(urlNode);
					}
				}
			}

			return fileURLs;

		}
		
		public ArrayList<byte[]> get2dMap(int index, String targetFileNameWithoutExt) {
			
			ArrayList<byte[]> map = null;
			
			JsonNode objNode = json.getArrayNode().get(index);
			
			if (JSON_MAPS.matches(objNode)) {
				Map<JsonStringNode,JsonNode> maps = JSON_MAPS.getValue(objNode);
				Set<JsonStringNode> keys = maps.keySet();
				if (keys.size() == 2) {
					map = new ArrayList<byte[]>(2);
					Iterator<JsonStringNode> it = keys.iterator();
					JsonStringNode key1 = it.next();
					JsonStringNode key2 = it.next();
					
					// TODO fix database HTTP response to send binary files instead of strings (after workshop?)
					
					String pgm = "";
					String pgmTmp = null;
					if (key1.getText().equals("pgm")) {
						pgmTmp = maps.get(key1).getText();
					} else if (key2.getText().equals("pgm")) {
						pgmTmp = maps.get(key2).getText();
					} else {
						System.out.println("unexpected response from RoboEarth server ('pgm' missing)");
						return null;
					}
					
					if (!pgmTmp.startsWith("P5")) {
						System.out.println("2d map received is not a pgm file");
						return null;
					}
					StringTokenizer st = new StringTokenizer(pgmTmp, "\n");
					int count = 0;
					while (st.hasMoreTokens()) {
						String s = st.nextToken();
						if (s.trim().startsWith("#")) {
							continue;
						}
						
						pgm += s+"\n";
						
						if (++count == 3) {
							break;
						}
					}

					try {
						byte[] pgmBytes = pgm.getBytes("ASCII");
						
						if (st.hasMoreTokens()) {

							String pixels = st.nextToken().trim();
							st = new StringTokenizer(pixels, " ");
							
							int length = st.countTokens() + pgmBytes.length;
							byte[] tmp = pgmBytes;
							pgmBytes = new byte[length];
							System.arraycopy(tmp, 0, pgmBytes, 0, tmp.length);
							
							int pixIndex = tmp.length;
							while (st.hasMoreTokens()) {
								String t = st.nextToken();
								if (t.equals("0")) {
									pgmBytes[pixIndex++] = 0;									
								} else if (t.equals("205")) {
									pgmBytes[pixIndex++] = -51;
								} else if (t.equals("254")) {
									pgmBytes[pixIndex++] = -1;
								} else {
									System.out.println("unknown pixel value: "+t);
									return null;
								}
							}
							
						} else {
							System.out.println("pgm file is incomplete");
							return null;
						}
						
						map.add(pgmBytes);
					} catch (UnsupportedEncodingException e) {
						System.out.println("'ASCII' encoding is not supported");
					}
					
					String yaml = null;
					if (key1.getText().equals("desc")) {
						yaml = maps.get(key1).getText();
					} else if (key2.getText().equals("desc")) {
						yaml = maps.get(key2).getText();
					} else {
						System.out.println("unexpected response from RoboEarth server ('desc' missing)");
						return null;
					}

					if (yaml.indexOf("proj2dmap.pgm") >= 0) {
						yaml = yaml.replace("proj2dmap.pgm", targetFileNameWithoutExt+".pgm");	
					} else if (yaml.indexOf("2dmap.pgm") >= 0) {
						yaml = yaml.replace("2dmap.pgm", targetFileNameWithoutExt+".pgm");
					} else {
						System.out.println("unexpected format of yaml file");
						return null;
					}
					
					try {
						map.add(yaml.getBytes("UTF8"));
					} catch (UnsupportedEncodingException e) {
						System.out.println("'UTF8' encoding is not supported");
					}
					
				}
			}
			
			return map;
			
		}
		
		@Override
		public int size() {
			return json.getArrayNode().size();
		}

	}

	/**
	 * 
	 * A RobotList object contains a JSON parser, that tries to extract the <tt>srdl</tt>
	 * and <tt>picture</tt> fields of a given JSON encoded string and presents the 
	 * collected data in a convenient Java AbstractList derived object. 
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	public static class RobotList extends AbstractList<String> {

		private final JsonRootNode json;

		// pre-built JSON node selectors, used to extract specific nodes
		private final static JsonNodeSelector<JsonNode, String> JSON_SRDL = 
				JsonNodeSelectors.aStringNode("srdl");

		private final static JsonNodeSelector<JsonNode, String> JSON_PICTURE = 
				JsonNodeSelectors.aNumberNode("picture");

		private final static JsonNodeSelector<JsonNode, String> JSON_AUTHOR = 
				JsonNodeSelectors.aStringNode("author");

		private final static JsonNodeSelector<JsonNode, String> JSON_DESCRIPTION = 
				JsonNodeSelectors.aStringNode("description");

		private final static JsonNodeSelector<JsonNode, String> JSON_UID = 
				JsonNodeSelectors.aStringNode("id");

		public RobotList(String jsonText) throws InvalidSyntaxException {
			json = json_parser.parse(jsonText);
		}

		@Override
		public String get(int index) {
			JsonNode robot = json.getArrayNode().get(index);
			return JSON_SRDL.getValue(robot);
		}

		public String getPicture(int index) {
			JsonNode robot = json.getArrayNode().get(index);
			return JSON_PICTURE.getValue(robot);
		}
		
		public String getDescription(int index) {
			return JSON_DESCRIPTION.getValue(json.getArrayNode().get(index));
		}

		public String getAuthor(int index) {
			return JSON_AUTHOR.getValue(json.getArrayNode().get(index));
		}

		public String getUID(int index) {
			return JSON_UID.getValue(json.getArrayNode().get(index));
		}

		@Override
		public int size() {
			return json.getArrayNode().size();
		}

	}
	
}
