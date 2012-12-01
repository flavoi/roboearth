/* \file MapExtractorServer.java
 * \brief The local 2d map extraction service implementation
 *
 * The MapExtractorServer listens to ROS service calls in order to create 2d 
 * maps using the 2d map extractor tool (MapExtractor.cpp).
 * 
 * This file is part of the RoboEarth ROS package re_2dmap_extractor.
 * 
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2012 by 
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
 * \date 2012
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 */

package roboearth.wp5.maps;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.re_2dmap_extractor.srv.RequestLocMap;
import ros.pkg.re_2dmap_extractor.srv.RequestNavMap;
import ros.pkg.re_msgs.msg.RosFile;

public class MapExtractorServer {

	protected Ros ros;
	protected NodeHandle n;
	
	protected static String packagePath;
	protected static String tmpPath;
	protected static String binPath;
	
	public MapExtractorServer(Ros ros, NodeHandle n) throws RosException {
		
		this.ros = ros;
		this.n = n;
		
		this.n.advertiseService("/re_2dmap_extractor/request_loc_map", new RequestLocMap(), new RequestLocMapCallback());
		this.n.advertiseService("/re_2dmap_extractor/request_nav_map", new RequestNavMap(), new RequestNavMapCallback());
		
		ros.logInfo("Waiting for service calls.");
		
	}

	/**
	 * 
	 * The callback class for the /re_2dmap_extractor/request_loc_map ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class RequestLocMapCallback implements ServiceServer.Callback<RequestLocMap.Request, RequestLocMap.Response> {

		@Override
		public RequestLocMap.Response call(RequestLocMap.Request req) {

			RequestLocMap.Response res = new RequestLocMap.Response();
			res.success = false;

			if (writeRosFile(tmpPath, req.octoMap)) {

				ProcessBuilder pb = new ProcessBuilder("./extract2dMap",
						"-z", ""+req.z,
						"-o", tmpPath + req.octoMap.name,
						"-n", tmpPath + req.targetMapName);
				
				pb = pb.directory(new File(binPath));
				
				try {
					Process p = pb.start();
					if (p.waitFor() == 0) {

						String mapName = req.targetMapName + ".pgm";
						String metaName = req.targetMapName + ".yaml";
						
						byte[] mapBytes = readFile(new File(tmpPath + mapName));
						byte[] metaBytes = readFile(new File(tmpPath + metaName));
						
						if (mapBytes != null && metaBytes != null) {
							res.locMap.data = mapBytes;
							res.locMap.name = mapName;
							res.locMeta.data = metaBytes;
							res.locMeta.name = metaName;
							res.success = true;
						}

					}
				} catch (IOException e) {
					e.printStackTrace();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				
			} else {
				System.out.println("Couldn't write Octomap '"+req.octoMap.name
						+ "' to path '" + tmpPath + "'.");
			}
			
			if (res.success) {
				ros.logInfo("RequestLocMap (octomap '" + req.octoMap.name + "'): Done");
			} else {
				ros.logInfo("RequestLocMap (octomap '" + req.octoMap.name + "'): Failed");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_2dmap_extractor/request_nav_map ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class RequestNavMapCallback implements ServiceServer.Callback<RequestNavMap.Request, RequestNavMap.Response> {

		@Override
		public RequestNavMap.Response call(RequestNavMap.Request req) {

			RequestNavMap.Response res = new RequestNavMap.Response();
			res.success = false;

			if (writeRosFile(tmpPath, req.octoMap)) {
				
				ProcessBuilder pb = new ProcessBuilder("./extract2dMap",
						"-z", ""+req.minZ,
						"-Z", ""+req.maxZ,
						"-o", tmpPath + req.octoMap.name,
						"-n", tmpPath + req.targetMapName);
				
				pb.directory(new File(binPath));
				
				try {
					Process p = pb.start();
					if (p.waitFor() == 0) {

						String mapName = req.targetMapName + ".pgm";
						String metaName = req.targetMapName + ".yaml";
						
						byte[] mapBytes = readFile(new File(tmpPath + mapName));
						byte[] metaBytes = readFile(new File(tmpPath + metaName));
						
						if (mapBytes != null && metaBytes != null) {
							res.navMap.data = mapBytes;
							res.navMap.name = mapName;
							res.navMeta.data = metaBytes;
							res.navMeta.name = metaName;
							res.success = true;
						}

					}
				} catch (IOException e) {
					e.printStackTrace();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				
			} else {
				System.out.println("Couldn't write Octomap '"+req.octoMap.name
						+ "' to path '" + tmpPath + "'.");
			}
			
			if (res.success) {
				ros.logInfo("RequestNavMap (octomap '" + req.octoMap.name + "'): Done");
			} else {
				ros.logInfo("RequestNavMap (octomap '" + req.octoMap.name + "'): Failed");
			}

			return res;

		}

	}

	public static String getLocalPackagePath(String packageName) {

		String dir = null;

		try {	

			// try to find package using rospack
			String line;
			String rospack = System.getenv().get("ROS_ROOT")+"/bin/rospack";
			Process get_path = new ProcessBuilder( rospack, "find", packageName).start();

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
	
	public static byte[] readFile(File file) {
	    
	    byte[] data = null;
		
	    if (file == null) {
	    	System.out.println("Error while trying to get file content: file is null");
	    	return data;
	    }
	    
	    long length = file.length();
	    if (length > Integer.MAX_VALUE) {
			System.out.println("  File '"+file.getName()+"' is too big! Operation canceled.");
	    } else {

	    	InputStream is = null;
	    	
	    	try {

		    	is = new FileInputStream(file);
			    data = new byte[(int)length];

			    int offset = 0;
			    int numRead = 0;
			    while (offset < data.length
			           && (numRead=is.read(data, offset, data.length-offset)) >= 0) {
			        offset += numRead;
			    }

			    if (offset < data.length) {
			    	data = null;
			        System.out.println("  Could not completely read file "+file.getName());
			    }
	    		
	    	} catch (FileNotFoundException e) {
				System.out.println("  Couldn't find file '"+ file.getName() +
						"'! Operation canceled.");
			} catch (IOException e) {
				System.out.println("  IOException occurred, while reading " +
						"from file '"+file.getName()+"'! Operation canceled.");
			} finally {
	    		if (is != null) {
	    			try {
						is.close();
					} catch (IOException e) {
					}	
	    		}
	    	}
		    
	    }
	    
	    return data;
	    
	}
	
	public static boolean writeRosFile(String targetPath, RosFile content) {

		boolean ok = false;

		if (content != null && targetPath != null && targetPath.length() > 0 
				&& content.name != null && content.name.length() > 0) {

			BufferedOutputStream bos = null;

			if (!targetPath.endsWith(File.separator)) {
				targetPath += File.separator;
			}
			
			try {
				FileOutputStream fos;
				fos = new FileOutputStream(new File(targetPath+content.name));
				bos = new BufferedOutputStream(fos);
				bos.write(content.data);
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
	
	public static void main(String[] args) {

		// Get local paths
		packagePath = getLocalPackagePath("re_2dmap_extractor");
		if (packagePath == null) {
			System.out.println("Couldn't determine local package path.");
			return;
		}
		tmpPath = packagePath + "tmp" + File.separator;
		binPath = packagePath + "bin" + File.separator;
		
		// Initialize rosjava 
		Ros ros = Ros.getInstance();
		ros.init("re_2dmap_extractor_server");

		// Create a NodeHandle
		NodeHandle n = ros.createNodeHandle();
		
		// Start map extractor server 
		try {
			
			new MapExtractorServer(ros, n);
			
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
