/* \file Inference.java
 * \brief A collection of inference tasks to be executed on the client (robot).
 *
 * This file is part of the RoboEarth ROS re_comm package.
 * 
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by 
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
 * \date 2011
 */

package roboearth.wp5.owl;

import java.util.ArrayList;
import java.util.Set;

import javax.vecmath.Matrix4d;

import org.semanticweb.HermiT.Reasoner;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.reasoner.NodeSet;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

public class Inference {

	public static Matrix4d getPose(OWLOntology srdl, String linkIRI) {
		
		if (srdl == null || linkIRI == null || linkIRI.length() == 0) {
			System.out.println("  Error: parameter error at call to Inference.getPose()");
			return null;
		}
		
		Matrix4d pose = null;
		
		System.out.print("  Initializing reasoner... ");
		Reasoner hermit = new Reasoner(srdl);
		System.out.println("DONE");
		
		System.out.print("  Checking SRDL consistency... ");
		if (!hermit.isConsistent()) {
			System.out.println("FAILED");
		} else {
			System.out.println("DONE");

			DefaultPrefixManager pm = IRIDepot.PREFIX_MANAGER;
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();
			
			// find robot instance
			OWLClass owlClass = factory.getOWLClass("knowrob:Robot", pm);
			NodeSet<OWLNamedIndividual> robotNodes;
			robotNodes = hermit.getInstances(owlClass, false);
			Set<OWLNamedIndividual> robotSet = robotNodes.getFlattened();
			if (robotSet.size() > 1) {
				System.out.println("  Error: " + robotSet.size() + " robot instances found (expected only one)");
			} else if (robotSet.size() == 0) {
				System.out.println("  Error: No robot instance found");
			} else {
				OWLNamedIndividual robot = robotSet.iterator().next();
				System.out.println("  IRI of robot instance: "+robot.getIRI());

				// find laser scanner instance
				IRI slIRI = IRI.create(linkIRI);
				OWLNamedIndividual targetLink;
				targetLink = factory.getOWLNamedIndividual(slIRI);
				if (!hermit.isDefined(targetLink)) {
					System.out.println("  Error: individual for target link '" + linkIRI + "' doesn't exist");
				} else {
					System.out.println("  IRI of target link: " + slIRI);
					
					// find UrdfJoint instance, that has the laser scanner 
					// as its succeeding link 
					OWLObjectProperty succLink;
					succLink = factory.getOWLObjectProperty("srdl2-comp:succeedingLink", pm);
	
					OWLClass urdfJoint = factory.getOWLClass("srdl2-comp:UrdfJoint", pm);
					NodeSet<OWLNamedIndividual> joints;
					joints = hermit.getInstances(urdfJoint, false);
					
					OWLNamedIndividual targetJoint = null;
					for (OWLNamedIndividual joint : joints.getFlattened()) {
						if (joint.hasObjectPropertyValue(succLink, targetLink, srdl)) {
							targetJoint = joint;
							break;
						}
					}
					// target link exists but there's no joint connecting it
					if (targetJoint == null) {
						Matrix4d identity = new Matrix4d();
						identity.setIdentity();
						return identity;
					}
					System.out.println("  Joint leading to target link: "+targetJoint);

					// find the related rotational matrix
					OWLObjectProperty orientation;
					orientation = factory.getOWLObjectProperty("knowrob:orientation", pm);
					
					Set<OWLIndividual> matrices;
					matrices = targetJoint.getObjectPropertyValues(orientation, srdl);
					if (matrices.size() > 1) {
						System.out.println("  Error: " + matrices.size() + " rotational matrices found (expected only one)");
					} else if (matrices.size() == 0) {
						System.out.println("  Error: No rotational matrix found");
					} else {
						OWLIndividual matrix = matrices.iterator().next();
						System.out.println("  Orientation of target link: " + matrix.toString());
						
						ArrayList<Matrix4d> rotMats = new ArrayList<Matrix4d>();
						do {

							// read in matrix
							OWLDataProperty m00 = factory.getOWLDataProperty("knowrob:m00", pm);
							OWLDataProperty m01 = factory.getOWLDataProperty("knowrob:m01", pm);
							OWLDataProperty m02 = factory.getOWLDataProperty("knowrob:m02", pm);
							OWLDataProperty m03 = factory.getOWLDataProperty("knowrob:m03", pm);
							OWLDataProperty m10 = factory.getOWLDataProperty("knowrob:m10", pm);
							OWLDataProperty m11 = factory.getOWLDataProperty("knowrob:m11", pm);
							OWLDataProperty m12 = factory.getOWLDataProperty("knowrob:m12", pm);
							OWLDataProperty m13 = factory.getOWLDataProperty("knowrob:m13", pm);
							OWLDataProperty m20 = factory.getOWLDataProperty("knowrob:m20", pm);
							OWLDataProperty m21 = factory.getOWLDataProperty("knowrob:m21", pm);
							OWLDataProperty m22 = factory.getOWLDataProperty("knowrob:m22", pm);
							OWLDataProperty m23 = factory.getOWLDataProperty("knowrob:m23", pm);
							OWLDataProperty m30 = factory.getOWLDataProperty("knowrob:m30", pm);
							OWLDataProperty m31 = factory.getOWLDataProperty("knowrob:m31", pm);
							OWLDataProperty m32 = factory.getOWLDataProperty("knowrob:m32", pm);
							OWLDataProperty m33 = factory.getOWLDataProperty("knowrob:m33", pm);
							
							double[] m = new double[16]; // TODO error handling
							m[0] = matrix.getDataPropertyValues(m00, srdl).iterator().next().parseDouble();
							m[1] = matrix.getDataPropertyValues(m01, srdl).iterator().next().parseDouble();
							m[2] = matrix.getDataPropertyValues(m02, srdl).iterator().next().parseDouble();
							m[3] = matrix.getDataPropertyValues(m03, srdl).iterator().next().parseDouble();
							m[4] = matrix.getDataPropertyValues(m10, srdl).iterator().next().parseDouble();
							m[5] = matrix.getDataPropertyValues(m11, srdl).iterator().next().parseDouble();
							m[6] = matrix.getDataPropertyValues(m12, srdl).iterator().next().parseDouble();
							m[7] = matrix.getDataPropertyValues(m13, srdl).iterator().next().parseDouble();
							m[8] = matrix.getDataPropertyValues(m20, srdl).iterator().next().parseDouble();
							m[9] = matrix.getDataPropertyValues(m21, srdl).iterator().next().parseDouble();
							m[10] = matrix.getDataPropertyValues(m22, srdl).iterator().next().parseDouble();
							m[11] = matrix.getDataPropertyValues(m23, srdl).iterator().next().parseDouble();
							m[12] = matrix.getDataPropertyValues(m30, srdl).iterator().next().parseDouble();
							m[13] = matrix.getDataPropertyValues(m31, srdl).iterator().next().parseDouble();
							m[14] = matrix.getDataPropertyValues(m32, srdl).iterator().next().parseDouble();
							m[15] = matrix.getDataPropertyValues(m33, srdl).iterator().next().parseDouble();
							
							Matrix4d mat4d = new Matrix4d(m);
							rotMats.add(mat4d);
							
							// search for relativeTo object property
							OWLObjectProperty relativeTo;
							relativeTo = factory.getOWLObjectProperty("knowrob:relativeTo", pm);
							
							Set<OWLIndividual> precMatrices;
							precMatrices = matrix.getObjectPropertyValues(relativeTo, srdl);
							if (precMatrices.size() > 1) {
								System.out.println("  Error: " + precMatrices.size() + " 'knowrob:relativeTo' data properties found (expected only one)");
								matrix = null;
							} else if (precMatrices.size() == 0) {
								matrix = null;
							} else {
								matrix = precMatrices.iterator().next();
								System.out.println("    relative to " + matrix);
							}
							
						} while (matrix != null);
						
						if (rotMats.size() > 0) {
							Matrix4d result = rotMats.get(rotMats.size()-1);
							for (int i = rotMats.size()-2; i>=0; i--) {
								result.mul(rotMats.get(i));
							}
							pose = result;
						}
						
					}
					
				}
				
			}

		}

		return pose;
		
	}
	
	public static double getZCoordinate(OWLOntology srdl, String linkIRI) {

		Matrix4d pose = getPose(srdl, linkIRI);
		if (pose == null) {
			System.out.println("  z coordinate of '" +linkIRI+ "': unknown");
			return Double.NaN;
		} else {
			System.out.println("  z coordinate of '" +linkIRI+ "': "+pose.m23);
			return pose.m23;
		}
		
	}
	
	/**
	 * For testing purposes only 
	 */
	public static void main(String[] args) {
		
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_robot1";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_base_link";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_base_laser_link";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_head_plate_frame";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_double_stereo_link";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_l_wrist_roll_link";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_l_gripper_led_frame";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_l_forearm_cam_frame";
		//String targetLink = "http://ias.cs.tum.edu/kb/PR2.owl#pr2_l_forearm_cam_optical_frame";
		String targetLink = "http://ias.cs.tum.edu/kb/amigo.owl#amigo_base_laser";
		
		//String fileName = "/home/alex/re_urdf2srdl/example/pr2.owl";
		String fileName = "/home/alex/re_urdf2srdl/example/amigo.owl";
		
		
		System.out.print("  Loading SRDL file... ");
		OWLOntology srdl = OWLIO.loadOntologyFromFile(fileName);
		if (srdl == null) {
			System.out.println("FAILED");
		} else {
			System.out.println("DONE");
			double z = getZCoordinate(srdl, targetLink);
			if (Double.isNaN(z)) {
				System.out.println("\n  Couldn't determine z coordinate of target link");
			} else {
				System.out.println("\n  z coordinate of target link: "+z);	
			}
		}
		
	}
	
}
