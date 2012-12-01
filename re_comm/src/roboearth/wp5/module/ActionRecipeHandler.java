/* \file ActionRecipeHandler.java
 * \brief Action recipe handler
 *
 * The action recipe handler provides the callbacks for ROS services related 
 * to action recipes.
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
package roboearth.wp5.module;

import java.util.ArrayList;

import org.semanticweb.owlapi.model.OWLOntology;

import roboearth.wp5.conn.REInterface;
import roboearth.wp5.owl.OWLIO;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.re_srvs.srv.DelActionRecipe;
import ros.pkg.re_srvs.srv.GetActionRecipe;
import ros.pkg.re_srvs.srv.QueryActionRecipes;
import ros.pkg.re_srvs.srv.SearchActionRecipes;
import ros.pkg.re_srvs.srv.SetActionRecipe;
import ros.pkg.re_srvs.srv.UpdateActionRecipe;

public class ActionRecipeHandler extends AbstractHandler {

	/**
	 * Constructor. Advertises the provided ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public ActionRecipeHandler(Ros ros, NodeHandle n) throws RosException {

		super(ros, n);

		this.n.advertiseService("/re_comm/get_action_recipe", new GetActionRecipe(), new GetActionRecipeCallback());
		this.n.advertiseService("/re_comm/set_action_recipe", new SetActionRecipe(), new SetActionRecipeCallback());
		this.n.advertiseService("/re_comm/del_action_recipe", new DelActionRecipe(), new DelActionRecipeCallback());
		this.n.advertiseService("/re_comm/update_action_recipe", new UpdateActionRecipe(), new UpdateActionRecipeCallback());
		this.n.advertiseService("/re_comm/search_action_recipes", new SearchActionRecipes(), new SearchActionRecipesCallback());
		this.n.advertiseService("/re_comm/query_action_recipes", new QueryActionRecipes(), new QueryActionRecipesCallback());

		ros.logInfo("Module 'ActionRecipeHandler' loaded.");

	}

	/**
	 * 
	 * The callback class for the /re_comm/get_action_recipe ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class GetActionRecipeCallback implements ServiceServer.Callback<GetActionRecipe.Request, GetActionRecipe.Response> {

		@Override
		public GetActionRecipe.Response call(GetActionRecipe.Request req) {

			GetActionRecipe.Response res = new GetActionRecipe.Response();
			REInterface con = getREConnection(guestInterfaceKey);
			String ont = con.requestActionRecipe(req.recipeUID);
			if (ont != null) {
				res.recipe = ont;
				res.success = true;
				ros.logInfo("GetActionRecipe (UID: " + req.recipeUID + "): Done");
			} else {
				res.success = false;
				ros.logInfo("GetActionRecipe (UID: " + req.recipeUID + "): Failed");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/set_action_recipe ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SetActionRecipeCallback implements ServiceServer.Callback<SetActionRecipe.Request, SetActionRecipe.Response> {

		@Override
		public SetActionRecipe.Response call(SetActionRecipe.Request req) {

			SetActionRecipe.Response res = new SetActionRecipe.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				OWLOntology ont = OWLIO.loadOntologyFromString(req.recipe);
				res.success = con.submitActionRecipe(ont, req.cls, req.id, req.description);
				if (res.success) {
					ros.logInfo("SetActionRecipe (UID: " + req.cls+"."+req.id + "): Done");
				} else {
					ros.logInfo("SetActionRecipe (UID: " + req.cls+"."+req.id + "): Failed");
				}

			} else {
				ros.logError("SetActionRecipe (UID: " + req.cls+"."+req.id + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/del_action_recipe ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class DelActionRecipeCallback implements ServiceServer.Callback<DelActionRecipe.Request, DelActionRecipe.Response> {

		@Override
		public DelActionRecipe.Response call(DelActionRecipe.Request req) {

			DelActionRecipe.Response res = new DelActionRecipe.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				res.success = con.deleteActionRecipe(req.recipeUID);
				if (res.success) {
					ros.logInfo("DelActionRecipe (UID: " + req.recipeUID + "): Done");
				} else {
					ros.logInfo("DelActionRecipe (UID: " + req.recipeUID + "): Failed");
				}

			} else {
				ros.logError("DelActionRecipe (UID: " + req.recipeUID + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/update_action_recipe ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class UpdateActionRecipeCallback implements ServiceServer.Callback<UpdateActionRecipe.Request, UpdateActionRecipe.Response> {

		@Override
		public UpdateActionRecipe.Response call(UpdateActionRecipe.Request req) {

			UpdateActionRecipe.Response res = new UpdateActionRecipe.Response();
			res.success = false;

			if (req.apiKey != null && req.apiKey.length() > 0) {

				REInterface con = getREConnection(req.apiKey);
				OWLOntology ont = OWLIO.loadOntologyFromString(req.recipe);
				res.success = con.updateActionRecipe(req.uid, ont, req.description);
				if (res.success) {
					ros.logInfo("UpdateActionRecipe (UID: " + req.uid + "): Done");
				} else {
					ros.logInfo("UpdateActionRecipe (UID: " + req.uid + "): Failed");
				}

			} else {
				ros.logError("UpdateActionRecipe (UID: " + req.uid + "): API key is missing!");
			}

			return res;

		}

	}

	/**
	 * 
	 * The callback class for the /re_comm/search_action_recipes ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class SearchActionRecipesCallback implements ServiceServer.Callback<SearchActionRecipes.Request, SearchActionRecipes.Response> {

		@Override
		public SearchActionRecipes.Response call(SearchActionRecipes.Request req) {

			SearchActionRecipes.Response res = new SearchActionRecipes.Response();
			res.success = false;
			
			REInterface con = getREConnection(guestInterfaceKey);
			ArrayList<String> uids = new ArrayList<String>();
			String[] onts = con.searchActionRecipes(req.searchID, uids);
			if (onts != null) {
				for (String ont : onts) {
					res.recipes.add(ont);	
				}
				res.uids = uids;
				res.success = true;
				ros.logInfo("SearchActionRecipes (search: " + req.searchID + "): Done");				
			} else {
				res.success = false;
				ros.logInfo("SearchActionRecipes (search: " + req.searchID + "): Failed");
			}

			return res;

		}

	}
	
	/**
	 * 
	 * The callback class for the /re_comm/query_action_recipes ROS service.
	 * 
	 * @author Alexander Perzylo, perzylo@cs.tum.edu
	 *
	 */
	class QueryActionRecipesCallback implements ServiceServer.Callback<QueryActionRecipes.Request, QueryActionRecipes.Response> {

		@Override
		public QueryActionRecipes.Response call(QueryActionRecipes.Request req) {

			QueryActionRecipes.Response res = new QueryActionRecipes.Response();
			res.result = "";

			REInterface con = getREConnection(guestInterfaceKey);
			String result = con.queryActionRecipeDB(req.query);
			if (result != null) {
				res.result = result;
				ros.logInfo("QueryActionRecipe (query:\n" + req.query + "): Done");
			} else {
				ros.logInfo("QueryActionRecipe (query:\n" + req.query + "): Failed");
			}

			return res;

		}

	}
	
}
