/* \file UnizarRoboEarthInterface.java
 * \brief Unizar RoboEarth interface
 *
 * The interface between Unizar's perception code and RoboEarth.
 * 
 * This file is part of the RoboEarth ROS re_comm package.
 * 
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2010 by 
 * <a href=" mailto:riazuelo@unizar.es">Luis Riazuelo</a>
 * University of Zaragoza
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
 * \author Luis Riazuelo
 * \version 1.0
 * \date 2010
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 */
package roboearth.wp1;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashSet;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

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
 * Interface between the Unizar Object Detector system (http://www.roboearth.org/wiki/Re_vision)
 * and the RoboEarth database.
 * 
 * This class copies the CopRoboEarthInterface and provides ROS services to upload object 
 * models to the UniZar database as well as to retrieve object models from the database 
 * and add them to re_vision.
 * 
 * @author Luis Riazuelo, riazuelo@unizar.es
 *
 */

public class UnizarRoboEarthInterface {

	/**
	 * Reference to rosjava
	 */
	protected Ros ros;

	/**
	 * Node handle used to advertise the ROS services
	 */
	protected NodeHandle n;

	/**
	 * Interface to the RoboEarth database
	 */
	protected REConnectionHadoop re_interface;

	protected SimpleDateFormat sdf = new SimpleDateFormat("yy-MM-dd_HH-mm-ss-SSS");

	/**
	 * Constructor. Advertises the needed ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public UnizarRoboEarthInterface(Ros ros, NodeHandle n) throws RosException {

		this.ros = ros;
		this.n = n;

		this.re_interface = new REConnectionHadoop("6e6574726f6d40b699e442ebdca5850e7cb7486679768aec3c70");

		n.advertiseService("/re_cop_interface/exportCopModel",         new RoboEarthExportCopModel(),    new ExportCopModelCallback());
		n.advertiseService("/re_cop_interface/retrieveCopModelByName", new RoboEarthRetrieveCopModel(),  new RetrieveCopModelCallback());

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

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(IRIDepot.ROBOEARTH));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(IRIDepot.KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);

			// Get classes from the KnowRob ontology
			OWLClass clsRoboEarthObjRecModelPlanar = factory.getOWLClass(IRIDepot.ROBOEARTH_OBJ_REC_MODEL_PLANAR, pm);
			OWLClass clsClass = factory.getOWLClass(IRIDepot.OWL_CLASS, pm);

			// Create classes as NamedIndividual of type owl:Class (required for providesModelFor relation)
			OWLNamedIndividual clsKRCup = factory.getOWLNamedIndividual(IRIDepot.KNOWROB_CUP, pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(clsClass, clsKRCup));
			OWLNamedIndividual clsRECup = factory.getOWLNamedIndividual(IRIDepot.ROBOEARTH_CUP, pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(clsClass, clsRECup));


			// object properties
			OWLObjectProperty objSubClassOf = factory.getOWLObjectProperty(IRIDepot.RDFS_SUB_CLASS_OF, pm);
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(objSubClassOf, clsRECup, clsKRCup));

			OWLNamedIndividual objModelInst = factory.getOWLNamedIndividual("ObjModelWorkshop0710", pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(clsRoboEarthObjRecModelPlanar, objModelInst));

			//OWLObjectProperty objCreatedByAlgorithm = factory.getOWLObjectProperty(IRIDepot.ROBOEARTH_CREATED_BY_ALGORITHM, pm);
			OWLObjectProperty objProvidesModelFor = factory.getOWLObjectProperty(IRIDepot.ROBOEARTH_PROVIDES_MODEL_FOR, pm);

			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(objProvidesModelFor, objModelInst, clsRECup));


			// data properties
			OWLDataProperty dataLinkToRecMod  = factory.getOWLDataProperty(IRIDepot.ROBOEARTH_LINK_TO_RECOGNITION_MODEL, pm);
			OWLDataProperty dataLinkToImgData = factory.getOWLDataProperty(IRIDepot.ROBOEARTH_LINK_TO_IMAGE_DATA, pm);
			OWLDataProperty dataCreationDate  = factory.getOWLDataProperty(IRIDepot.ROBOEARTH_CREATION_DATE_TIME, pm);

			// TODO: perform mapping of values to the KnowRob notation
			// TODO: fill in these values once the URL of uploaded images/models is known
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(dataCreationDate,  objModelInst, ""));
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(dataLinkToRecMod,  objModelInst, ""));
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(dataLinkToImgData, objModelInst, ""));


		} catch (Exception e) {
			ontology = null;
			e.printStackTrace();
		}

		return ontology;

	}



	/**
	 * Create a ZIP archive from a list of files.
	 * 
	 * @param filenames List of files to be zipped
	 * @param target File name of the resulting zip archive
	 */
	public static void createZipFromFiles(String xmlFile, Set<String> filenames, String target) {

		byte[] buffer = new byte[1024];
		try {

			ZipOutputStream out = new ZipOutputStream(new FileOutputStream(target));

			FileInputStream in = new FileInputStream(xmlFile);
			out.putNextEntry(new ZipEntry(new File(xmlFile).getName()));

			int size;
			while ((size = in.read(buffer)) > 0) {
				out.write(buffer, 0, size);
			}

			out.closeEntry();
			in.close();

			// add other files
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
	 * 
	 * The callback class for the re_export_cop_model ROS service.
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 * @modify Luis Riazuelo, riazuelo@unizar.es
	 * @modify Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class ExportCopModelCallback implements ServiceServer.Callback<RoboEarthExportCopModel.Request, RoboEarthExportCopModel.Response> {

		@Override
		public RoboEarthExportCopModel.Response call(RoboEarthExportCopModel.Request req) {

			RoboEarthExportCopModel.Response res = new RoboEarthExportCopModel.Response();
			res.success = 0;

			if (req.object_id >0 ) {


				// send /cop/save request to CoP
				CommunicationVisApplet.visualizeCommunication("Retrieving model from the vision system... \n\nSending: /cop/save "+req.object_id, "", null, "cop.png");

				// just for nicer visualization:
				try {
					Thread.sleep(1500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}

				ServiceClient<cop_save.Request, cop_save.Response, cop_save> client = n.serviceClient("/cop/save", new ros.pkg.vision_srvs.srv.cop_save());
				cop_save.Request rq = new cop_save().createRequest();
				rq.object_id=req.object_id;		
				String modelFile = null;
				String imageFile = null;

				// get the file names of the model
				try {

					cop_save.Response resp = client.call(rq);

					String vis_string = resp.xmlfilename+"\n";
					String remoteXmlFile = resp.xmlfilename;
					File remoteParent = new File(remoteXmlFile).getParentFile();
					String remoteModelDir = remoteParent.getAbsolutePath()+"/";
					String modelName = remoteParent.getName()+"/";

					// copy the files from the remote computer
					String cop_uri = Util.getURIforService("/cop/save");
					cop_uri = cop_uri.substring(9, cop_uri.length()-7);

					File localModelDirFile = new File(Util.modelDir + modelName);
					if (localModelDirFile.exists()) {
						Util.deleteFolderRec(localModelDirFile, false);
					} else {
						localModelDirFile.mkdir();
					}
					String localModelDir = localModelDirFile.getAbsolutePath()+"/";

					Util.sshCopyFromRemote(remoteXmlFile, cop_uri, localModelDir);
					HashSet<String> filenames = new HashSet<String>();
					for(String f:resp.filenames) {
						Util.sshCopyFromRemote(remoteModelDir+f, cop_uri, localModelDir);

						filenames.add(localModelDir+"/"+f);
						vis_string+=f+"\n";

						// remember if image -> upload separately
						if(f.toLowerCase().endsWith(".png") || f.toLowerCase().endsWith(".jpg")) {
							imageFile = localModelDir + f;
						}
					}
					CommunicationVisApplet.visualizeCommunication("", vis_string, null, null);

					// zip the files (XML plus files referenced therein)
					modelFile = localModelDir + "re_obj_model_"+sdf.format(new Date())+".zip";
					createZipFromFiles(localModelDir + new File(remoteXmlFile).getName(), filenames, modelFile);

					// call cop to get the cop_descriptor model
					String[] model_info = CopROSClient.copModelTypeSemClassForID(req.object_id);

					// generate the OWL representation of the recognition model
					OWLOntology objectmodel = buildOWLObjectDescription(model_info[0], model_info[1]);


					// just for nicer visualization:
					try {
						Thread.sleep(1500);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}


					// upload the files to RoboEarth
					//CommunicationVisApplet.visualizeCommunication("Uploading files to RoboEarth... \nModel "+model_info[0]+"\nObject type: "+model_info[1], "", null, "roboearth.png");

					String uid = "";
					if(model_info[1].equals("cabinet1")) {
						uid="cabinetrecmodel";
					} else if(model_info[1].equals("bed1")) {
						uid="bedrecmodel";
					} else if(model_info[1].equals("bottle1")) {
						uid="bottlerecmodel";
					} else {
						uid=model_info[1];
					}

					ArrayList<File> fileList = new ArrayList<File>();
					if (imageFile != null) {
						fileList.add(new File(imageFile));
					}
					if (modelFile != null) {
						fileList.add(new File(modelFile));
					}

					if(re_interface.submitObject(objectmodel, uid, uid, "Cop object id: "+req.object_id, fileList)) {
						res.success = 1;	
					} else {
						System.err.println("ERROR: Failed to upload object model.");
					}

					//CommunicationVisApplet.visualizeCommunication("", "Object model upload finished.", null, null);


				} catch (Exception e) {

					e.printStackTrace();

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
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 * @modify Luis Riazuelo, riazuelo@unizar.es
	 * @modify Alexander Perzylo, perzylo@cs.tum.edu
	 */
	class RetrieveCopModelCallback implements ServiceServer.Callback<RoboEarthRetrieveCopModel.Request, RoboEarthRetrieveCopModel.Response> {

		@Override
		public RoboEarthRetrieveCopModel.Response call(RoboEarthRetrieveCopModel.Request req) {

			Publisher<ros.pkg.std_msgs.msg.String> pub = null;
			RoboEarthRetrieveCopModel.Response res = new RoboEarthRetrieveCopModel.Response();
			res.success = 0;
			if (req.object_name != null && req.object_name.length() > 0) {

				System.err.println("Retrieving "+ req.object_name);

				try {	

					// register to the /cop/new_signatures topic (where the new models will be written to)
					pub = n.advertise("/cop/new_signatures", new ros.pkg.std_msgs.msg.String(), 100);

					// retrieve object information from RoboEarth
					String targetFolder = Util.modelDir+"re_obj_model_"+sdf.format(new Date())+"/";

					File targetFolderFile = new File(targetFolder);
					if (targetFolderFile.exists()) {
						Util.deleteFolderRec(targetFolderFile, false);
					} else {
						targetFolderFile.mkdir();
					}

					CommunicationVisApplet.visualizeCommunication("Requesting model for object '"+req.object_name+"' from RoboEarth...", "", null, "roboearth.png");

					ArrayList<String> outFilenames = new ArrayList<String>();
					ArrayList<String> outFileURLs = new ArrayList<String>();

					String owldesc = re_interface.requestObject(req.object_name.toLowerCase(), outFilenames, outFileURLs);

					if (owldesc !=null) {

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

						CommunicationVisApplet.visualizeCommunication("", "Object model download finished.\n\n"+targetFolderFile.getName(), null, null);

						// just for nicer visualization:
						try {
							Thread.sleep(2000);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}

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

						String remote_cop_path = Util.getRemoteRosPackagePath(cop_uri, "re_gazebo_vslam");
						if (remote_cop_path == null) {
							System.err.println("Error: Couldn't find remote ROS package path for 're_gazebo_vslam' on '"+cop_uri+"'!");
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
						// send the path to the XML to /cop/new_signatures (which makes CoP load the model)
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

						System.err.println("published \n" + owldesc);


					} else {
						CommunicationVisApplet.visualizeCommunication("", "No model for "+req.object_name+" found.", null, null);
						ros.logWarn("CopRoboEarthInterface: No model for "+req.object_name+" found");
					}

				} catch(Exception e) {
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


	public static void main(String[] args) {


		// Initialize ros 
		Ros ros = Ros.getInstance();
		if (!ros.isInitialized()) {
			ros.init("RE_Unizar_interface");	
		}
		NodeHandle n = ros.createNodeHandle();

		try {

			new UnizarRoboEarthInterface(ros, n);

		} catch (RosException e) {

			ros.logFatal("Fatal error occurred. Shutting down!");
			n.shutdown();
			e.printStackTrace();
			return;

		}

		ros.spin();

	}


}
