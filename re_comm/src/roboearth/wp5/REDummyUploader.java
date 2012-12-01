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


import edu.tum.cs.ias.knowrob.utils.ros.*;


public class REDummyUploader {


	//
	// ugly method for conveniently resetting the database content to the state
	// before the update
	//
	public static void resetDBcontent() {
		
		System.out.println("Updating expedit: " + RosUtilities.rospackFind("re_ontology"));
		
		
		//REClients.updateObjectOWL(RosUtilities.rospackFind("re_ontology") + "/owl/cabinet_model.owl", "cabinet.ikeaexpedit2x4", "");
		System.out.println("Updating bedrecmodel");
		REClients.updateObjectOWL("/home/tenorth/re-obj-models/bed/bed.owl", "bedrecmodel.bedrecmodel", "");
		
		System.out.println("Updating bottlerecmodel");
		REClients.updateObjectOWL("/home/tenorth/re-obj-models/bottle/bottle.owl", "bottlerecmodel.bottlerecmodel", "");
		
		System.out.println("Updating expedit");
		REClients.updateObjectOWL("/home/tenorth/re-obj-models/expedit/expedit.owl", "cabinet.ikeaexpedit2x4", "");

		 
		
		//System.out.println("Updating ks map");
		//updateMap("/home/tenorth/re-obj-models/semantic-map-ks.owl", "semanticenvironmentmap.semanticenvironmentmap7635", "");
		
		//System.out.println("Updating fmi map");
		//updateMap("/home/tenorth/re-obj-models/semantic-map-fmi.owl", "semanticenvironmentmap.semanticenvironmentmap7398", "");
	}

	
	public static void main(String[] args) {
		
		if(args.length==0) {
			System.err.println("USAGE: reset_models [reset | update]");
		}
		
		if(args[0].equals("reset"))
		REDummyUploader.resetDBcontent();
	}
}
