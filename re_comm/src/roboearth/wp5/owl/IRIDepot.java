/* \file IRIDepot.java
 * \brief An IRI depot
 *
 * The IRIDepot stores the IRIs of used OWL classes and properties.
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
package roboearth.wp5.owl;

import org.semanticweb.owlapi.util.DefaultPrefixManager;

/**
 * 
 * The IRIDepot is a central place, where the IRIs of all used classes and properties are stored.
 * As they are abbreviated IRIs, they are intended to be used in conjunction with the prefix manager
 * given in this class as well. The central storage helps to avoid unexpected behavior due to
 * typographic errors. If an entity would have to be renamed, it only has to be done once in this 
 * class.
 * 
 * @author Alexander Perzylo, perzylo@cs.tum.edu
 *
 */
public class IRIDepot {

	// Base IRI for RoboEarth 
	public final static String ROBOEARTH = "http://www.roboearth.org/kb/roboearth.owl#";
	
	// Base IRI for KnowRob 
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";

	// Base IRI for OWL 
	public final static String OWL = "http://www.w3.org/2002/07/owl#";
	
	// Base IRI for OWL2-XML 
	public final static String OWL2XML = "http://www.w3.org/2006/12/owl2-xml#";
	
	// Base IRI for RDF
	public final static String RDF = "http://www.w3.org/1999/02/22-rdf-syntax-ns#";
	
	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for SRDL2 
	public final static String SRDL2="http://ias.cs.tum.edu/kb/srdl2.owl#";
	
	// Base IRI for SRDL2-CAP
	public final static String SRDL2_CAP = "http://ias.cs.tum.edu/kb/srdl2-cap.owl#";

	// Base IRI for SRDL2-COMP
	public final static String SRDL2_COMP = "http://ias.cs.tum.edu/kb/srdl2-comp.owl#";

	// Base IRI for XSD
	public final static String XSD = "http://www.w3.org/2001/XMLSchema#";

	
	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(IRIDepot.ROBOEARTH);
	static {
		PREFIX_MANAGER.setPrefix("roboearth:", ROBOEARTH);
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("owl:", OWL);
		PREFIX_MANAGER.setPrefix("owl2xml:", OWL2XML);
		PREFIX_MANAGER.setPrefix("rdf:", RDF);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
		PREFIX_MANAGER.setPrefix("srdl2:", SRDL2);
		PREFIX_MANAGER.setPrefix("srdl2-cap:", SRDL2_CAP);
		PREFIX_MANAGER.setPrefix("srdl2-comp:", SRDL2_COMP);
		PREFIX_MANAGER.setPrefix("xsd:", XSD);
	}
	
	// abbreviated IRIs of some RoboEarth classes
	public final static String ROBOEARTH_ROBOEARTH_RL_POLICY = "roboearth:RoboEarthRLPolicy";
	public final static String ROBOEARTH_RL_POLICY = "roboearth:ReinforcementLearningPolicy";
	public final static String ROBOEARTH_POLICY = "roboearth:Policy";
	public final static String ROBOEARTH_MAZE_LEARNING = "roboearth:RoboEarthMazeLearning";
	public final static String ROBOEARTH_OBJ_REC_MODEL_PLANAR = "roboearth:RoboEarthObjRecModelPlanar";
	public final static String ROBOEARTH_CUP = "roboearth:Cup";
	
	// abbreviated IRIs of some RoboEarth properties
	public final static String ROBOEARTH_CREATED_BY_ALGORITHM = "roboearth:createdByAlgorithm";
	public final static String ROBOEARTH_Q_MATRIX = "roboearth:qmatrix";
	public final static String ROBOEARTH_LINK_TO_RECOGNITION_MODEL = "roboearth:linkToRecognitionModel";
	public final static String ROBOEARTH_LINK_TO_IMAGE_DATA = "roboearth:linkToImageData";
	public final static String ROBOEARTH_CREATION_DATE_TIME = "roboearth:creationDateTime";
	public final static String ROBOEARTH_PROVIDES_MODEL_FOR = "roboearth:providesModelFor";
	
	// abbreviated IRIs of some KnowRob classes
	public final static String KNOWROB_ACTION = "knowrob:Action";
	public final static String KNOWROB_PARTIAL_ORDERING_STRICT = "knowrob:PartialOrdering-Strict";
	public final static String KNOWROB_VELOCITY_COMMAND = "knowrob:VelocityCommand";
	public final static String KNOWROB_MATRIX = "knowrob:Matrix";
	public final static String KNOWROB_CUP = "knowrob:Cup";

	// abbreviated IRIs of some KnowRob properties
	public final static String KNOWROB_SUBEVENTS = "knowrob:subEvents";
	public final static String KNOWROB_ORDERING_CONSTRAINTS = "knowrob:orderingConstraints";
	public final static String KNOWROB_OCCURS_BEFORE_IN_ORDERING = "knowrob:occursBeforeInOrdering";
	public final static String KNOWROB_OCCURS_AFTER_IN_ORDERING = "knowrob:occursAfterInOrdering";
	
	// abbreviated IRIs of some OWL classes
	public final static String OWL_CLASS = "owl:Class";

	// abbreviated IRIs of some RDFS properties
	public final static String RDFS_SUB_CLASS_OF = "rdfs:subClassOf";
	
}
