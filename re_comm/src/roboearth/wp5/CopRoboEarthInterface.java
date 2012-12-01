/* \file CopRoboEarthInterface.java
 * \brief CoP RoboEarth interface
 *
 * The interface between the CoP cognitive perception system and RoboEarth.
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

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashSet;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.AddImport;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLImportsDeclaration;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import roboearth.wp5.conn.REConnectionHadoop;
import roboearth.wp5.owl.IRIDepot;
import roboearth.wp5.util.Util;
import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.ServiceServer;
import ros.pkg.re_srvs.srv.RoboEarthExportCopModel;
import ros.pkg.re_srvs.srv.RoboEarthRetrieveCopModel;
import ros.pkg.vision_srvs.srv.cop_save;
import de.tum.in.fipm.kipm.gui.visualisation.applets.CommunicationVisApplet;
import edu.tum.cs.ias.knowrob.CopROSClient;

/**
 * 
 * Interface between the CoP cognitive perception system (http://www.ros.org/wiki/cop)
 * and the RoboEarth database.
 * 
 * Provides ROS services to upload CoP object models to the RoboEarth database as well
 * as to retrieve object models from the database and add them to CoP.
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */

public class CopRoboEarthInterface {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;
	protected REConnectionHadoop re_interface;


	/**
	 * Constructor. Advertises the needed ROS services.
	 * @param node_name String to be used as name for the ROS node
	 * @throws RosException if advertising ROS services failed
	 */
	public CopRoboEarthInterface(String node_name) throws RosException {

		initRos(node_name);

		this.re_interface = new REConnectionHadoop("6e6574726f6d40b699e442ebdca5850e7cb7486679768aec3c70");
		//this.re_interface = new REConnectionMySQL();

		n.advertiseService("/re_cop_interface/exportCopModel",         new RoboEarthExportCopModel(),    new ExportCopModelCallback());
		n.advertiseService("/re_cop_interface/retrieveCopModelByName", new RoboEarthRetrieveCopModel(),  new RetrieveCopModelCallback());

	}

	/**
	 * Initialize the ROS environment if it has not yet been initialized
	 * 
	 * @param node_name A unique node name
	 */
	protected static void initRos(String node_name) {

		ros = Ros.getInstance();

		if(!ros.isInitialized()) {
			ros.init(node_name);
		}
		n = ros.createNodeHandle();

	}


	/**
	 * 
	 * The callback class for the re_export_cop_model ROS service.
	 * 
	 * The callback method... 
	 * * first calls the Cop system (/cop/save service) to trigger the export 
	 * * retrieves the object model file via ssh
	 * * packs all files into a zip archive
	 * * creates an OWL representation describing this model
	 * * and uploads everything to RoboEarth
	 * 
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class ExportCopModelCallback implements ServiceServer.Callback<RoboEarthExportCopModel.Request, RoboEarthExportCopModel.Response> {

		@Override
		public RoboEarthExportCopModel.Response call(RoboEarthExportCopModel.Request req) {

			RoboEarthExportCopModel.Response res = new RoboEarthExportCopModel.Response();
			res.success = 0;

			if (req.object_id > 0 ) {

				// send /cop/save request to CoP
				CommunicationVisApplet.visualizeCommunication("Retrieving model from the vision system... \n\nSending: /cop/save "+req.object_id, "", null, "cop.png");

				ServiceClient<cop_save.Request, cop_save.Response, cop_save> client = n.serviceClient("/cop/save", new ros.pkg.vision_srvs.srv.cop_save());
				cop_save.Request rq = new cop_save().createRequest();
				rq.object_id=req.object_id;		
				File modelFile = null;
				File imageFile = null;

				// get the file names of the model
				try {

					cop_save.Response resp = client.call(rq);

					String vis_string = resp.xmlfilename+"\n";

					String remote_xmlFile = resp.xmlfilename;
					File remote_Parent = new File(remote_xmlFile).getParentFile();
					String remote_model_dir = remote_Parent.getAbsolutePath() + "/";
					String modelName = remote_Parent.getName()+"/";

					File local_model_dir_file  = new File(Util.modelDir + modelName);
					if (local_model_dir_file.exists()) {
						Util.deleteFolderRec(local_model_dir_file, false);
					} else {
						local_model_dir_file.mkdir();
					}
					String local_model_dir = local_model_dir_file.getAbsolutePath()+"/";

					System.err.println(remote_xmlFile);

					// copy the files from the remote computer
					String cop_uri = Util.getURIforService("/cop/save");
					// check for empty cop_uri (if server cannot be found)
					if(cop_uri.length()<16) {
						System.err.println("Error: Couldn't find remote computer running CoP!\n"+cop_uri);
						return res;
					}
					cop_uri = cop_uri.substring(9, cop_uri.length()-7);

					Util.sshCopyFromRemote(remote_xmlFile, cop_uri, local_model_dir);

					HashSet<String> local_filenames = new HashSet<String>();
					for(String f:resp.filenames) {

						System.err.println(f);

						File fl = new File(f);
						if(fl.isAbsolute())
							Util.sshCopyFromRemote(f, cop_uri, local_model_dir);
						else 
							Util.sshCopyFromRemote(remote_model_dir+f, cop_uri, local_model_dir);

						local_filenames.add(local_model_dir + fl.getName());
						vis_string+=f+"\n";

						// remember if image -> upload separately
						if(f.toLowerCase().endsWith(".png") || f.toLowerCase().endsWith(".jpg"))
							imageFile = new File(local_model_dir + f);
					}
					CommunicationVisApplet.visualizeCommunication("", vis_string, null, null);

					System.err.println("files copied");

					// zip the files (XML plus files referenced therein)
					SimpleDateFormat sdf = new SimpleDateFormat("yy-MM-dd_HH-mm-ss-SSS");
					String modelFileName = local_model_dir+"re_obj_model_"+sdf.format(new Date())+".zip";

					// add xml file to set of file names
					local_filenames.add(local_model_dir + new File(remote_xmlFile).getName());
					Util.createZipFromFiles(local_filenames, modelFileName);

					// call cop to get the cop_descriptor model
					System.err.println("Calling cop for model info...");
					String[] model_info = CopROSClient.copModelTypeSemClassForID(req.object_id);

					// generate the OWL representation of the recognition model
					System.err.println("Generating OWL description...");
					OWLOntology objectmodel = buildOWLObjectDescription(model_info[0], model_info[1]);

					System.err.println(objectmodel.toString());


					// upload the files to RoboEarth
					CommunicationVisApplet.visualizeCommunication("Uploading files to RoboEarth... \nModel "+model_info[0]+"\nObject type: "+model_info[1], "", null, "roboearth.png");

					modelFile = new File(modelFileName);
					ArrayList<File> fileList = new ArrayList<File>();
					if (imageFile != null) {
						fileList.add(imageFile);
					}
					if (modelFile != null) {
						fileList.add(modelFile);
					}

					if(re_interface.submitObject(objectmodel, "cop", model_info[1], "Cop object id:"+req.object_id, fileList)) {
						res.success = 1;
					} else {
						System.err.println("ERROR: Failed to upload object model.");
					}

					CommunicationVisApplet.visualizeCommunication("", "Object model upload finished.", null, null);


				} catch (RosException e) {
					CommunicationVisApplet.visualizeCommunication("", "Export of model for Cop ID "+req.object_id+" failed.", null, null);
					ros.logError("CopRoboEarthInterface: Export of model for Cop ID "+req.object_id+" failed");
				}
			}
			return res;
		}
	}




	/**
	 * 
	 * The callback class for the re_retrieve_cop_model ROS service.
	 * 
	 * The callback method...
	 * * downloads the model from RoboEarth
	 * * unpacks the zip archive 
	 * * copies the object model file to the computer where cop is running via ssh
	 * * announces the new model on the /cop/new_signatures topic
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class RetrieveCopModelCallback implements ServiceServer.Callback<RoboEarthRetrieveCopModel.Request, RoboEarthRetrieveCopModel.Response> {

		@Override
		public RoboEarthRetrieveCopModel.Response call(RoboEarthRetrieveCopModel.Request req) {

			Publisher<ros.pkg.std_msgs.msg.String> pub = null;
			RoboEarthRetrieveCopModel.Response res = new RoboEarthRetrieveCopModel.Response();
			res.success = 0;

			if (req.object_name != null && req.object_name.length() > 0) {

				// register to the /cop/new_signatures topic (where the new models will be written to)
				try {

					pub = n.advertise("/cop/new_signatures", new ros.pkg.std_msgs.msg.String(), 100);


					// retrieve object information from RoboEarth
					SimpleDateFormat sdf = new SimpleDateFormat("yy-MM-dd_HH-mm-ss-SSS");
					String targetFolder = Util.re_commDir + "download/re_obj_model_"+sdf.format(new Date())+"/";

					File targetFolderFile = new File(targetFolder);
					if (targetFolderFile.exists()) {
						Util.deleteFolderRec(targetFolderFile, false);
					} else {
						targetFolderFile.mkdir();
					}

					CommunicationVisApplet.visualizeCommunication("Requesting model for object '"+req.object_name+"' from RoboEarth...", "", null, "roboearth.png");

					ArrayList<String> outFilenames = new ArrayList<String>();
					ArrayList<String> outFileURLs = new ArrayList<String>();
					// only continue if an object model could be retrieved
					String owldesc = re_interface.requestObject(req.object_name, outFilenames, outFileURLs); 
					if (owldesc != null) {

						res.owldata = owldesc;

						File zipfile = null; 
						for (String fileName : outFilenames) {
							if (fileName.startsWith("re_obj_model_")) {
								zipfile = re_interface.requestObjectBinaryFile(req.object_name.toLowerCase(), fileName, targetFolder);
								break;
							}
						}

						if (zipfile == null) {
							System.err.println("Error: No object detection model file found for uid '"+req.object_name.toLowerCase()+"'!");
							return res;
						}

						CommunicationVisApplet.visualizeCommunication("", "Object model download finished.\n\n"+zipfile.getName(), null, null);

						// unzip zip file
						if (!Util.extractZipFile(zipfile.getAbsolutePath(), targetFolder)) {
							System.err.println("Error: Extracting files from zip file failed!");
							return res;
						} else {
							zipfile.delete();
						}

						// copy the files to the remote computer using scp before sending the link to CoP
						String cop_uri  = Util.getURIforService("/cop/save");

						// check for empty cop_uri (if server cannot be found)
						if(cop_uri.length()<16) {
							System.err.println("Error: Couldn't find remote computer running CoP!\n"+cop_uri);
							return res;
						}
						cop_uri = cop_uri.substring(9, cop_uri.length()-7);

						String remote_cop_path = Util.getRemoteRosPackagePath(cop_uri, "cop");
						if (remote_cop_path == null) {
							System.err.println("Error: Couldn't find remote ROS package path for 'cop' on '"+cop_uri+"'!");
							return res;
						} else {
							remote_cop_path += "/resource/";	
						}

						String xmlFileName = null;
						for (String filename : new File( targetFolder ).list()) {
							Util.sshCopyToRemote(targetFolder+filename, cop_uri, remote_cop_path);
							if(filename.toLowerCase().endsWith(".xml")) {
								if (xmlFileName == null) {
									xmlFileName = filename;	
								} else {
									System.err.println("Error: More than one XML files found in zip file!");
									return res;
								}
							}
						}

						if (xmlFileName != null) {
							ros.pkg.std_msgs.msg.String m = new ros.pkg.std_msgs.msg.String();
							m.data = remote_cop_path+"/"+xmlFileName;
							CommunicationVisApplet.visualizeCommunication("Sending model for object '"+req.object_name+"' to the vision system...\n\n"+xmlFileName, "", null, "cop.png");
							pub.publish(m);

							// just for nicer visualization:
							try {
								Thread.sleep(1500);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}

							CommunicationVisApplet.visualizeCommunication("", "Object model "+xmlFileName+" received.", null, null);

						} else {
							System.err.println("Error: No XML file found in zip file!");
							return res;
						}

						res.success = 1;

					} else {
						CommunicationVisApplet.visualizeCommunication("", "No model for "+req.object_name+" found.", null, null);
						ros.logWarn("CopRoboEarthInterface: No model for "+req.object_name+" found");
					}

				}catch(Exception e) {
					e.printStackTrace();
				} finally {
					if (pub != null) {
						pub.shutdown();
					}
				}

			} else {
				CommunicationVisApplet.visualizeCommunication("", "No model for "+req.object_name+" found.", null, null);
				ros.logWarn("CopRoboEarthInterface: No model for "+req.object_name+" found");
			}
			return res;

		}

	}



	/**
	 * Builds an OWL encoded representation of the CoP object model.
	 * @param objClass the object types
	 * @return the OWL encoded object description
	 */
	protected OWLOntology buildOWLObjectDescription(String modelType, String objClass) {

		OWLOntology ontology = null;
		try {

			// Create ontology manager and data factory
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();

			// Get prefix manager using the base IRI of the JoystickDrive ontology as default namespace
			DefaultPrefixManager pm = IRIDepot.PREFIX_MANAGER;
			pm.setPrefix("comp_cop:", "http://ias.cs.tum.edu/kb/comp_cop.owl#");

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(IRIDepot.ROBOEARTH));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(IRIDepot.KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);

			// Get classes from the KnowRob ontology
			OWLClass clsObjModel = factory.getOWLClass("comp_cop:" + modelType, pm);
			OWLClass clsClass = factory.getOWLClass(IRIDepot.OWL_CLASS, pm);

			// Create classes as NamedIndividual of type owl:Class (required for providesModelFor relation)
			OWLNamedIndividual clsObjType = factory.getOWLNamedIndividual("knowrob:" + objClass, pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(clsClass, clsObjType));


			// object model instance
			SimpleDateFormat sdf = new SimpleDateFormat("yy_MM_dd_HH_mm_ss_SSS");
			OWLNamedIndividual instObjModel = factory.getOWLNamedIndividual("comp_cop:" + modelType +sdf.format(new Date()), pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(clsObjModel, instObjModel));

			OWLObjectProperty propProvidesModelFor = factory.getOWLObjectProperty(IRIDepot.ROBOEARTH_PROVIDES_MODEL_FOR, pm);
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(propProvidesModelFor, instObjModel, clsObjType));


			// creation date
			OWLDataProperty dataCreationDate  = factory.getOWLDataProperty(IRIDepot.ROBOEARTH_CREATION_DATE_TIME, pm);
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(dataCreationDate,  instObjModel, sdf.format(new Date())));


		} catch (Exception e) {
			ontology = null;
			e.printStackTrace();
		}

		return ontology;

	}



	public static void main(String[] args) {

		try {
			new CopRoboEarthInterface("re_cop_interface");
		} catch (RosException e) {
			e.printStackTrace();
			return;
		}

		ros.spin();

	}


}
