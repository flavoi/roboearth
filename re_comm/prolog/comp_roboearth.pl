/** <module> comp_roboearth

  Description:
    Interface to the RoboEarth data base

    Prolog-wrapper functions around classes in re_comm to allow
    access to the database content from Prolog.


  Copyright (C) 2010-2011 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@license GPL
*/

:- module(comp_roboearth,
    [
      re_download_action_recipe/3,
      re_set_opening_radius/2,
      re_update_opening_radius/2,
      re_request_action_recipe/2,
      re_request_object/2,
      re_request_map/2,
      re_submit_object/4,
      re_submit_action_recipe/4,
      re_submit_map/4,
      re_update_object/3,
      re_update_action_recipe/3,
      re_update_map/3,
      re_start_map_server/2,
      re_kill_map_server/1,
      re_generate_cpl_plan/2,
      re_request_map_for/2
    ]).


:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('mod_vis')).
:- use_module(library('knowrob_actions')).


:-  rdf_meta
      re_download_action_recipe(r,r,r),
      re_set_opening_radius(r,-),
      re_update_opening_radius(r,-),
      re_request_model_for(r,-),
      re_download_model_for(r,-),
      re_request_recipe_for(+,r),
      re_download_recipe_for(+,r),
      re_request_action_recipe(r,r),
      re_request_object(r,r),
      re_request_map(r,r),
      re_submit_object(r,r,r,r),
      re_submit_action_recipe(r,r,r,r),
      re_submit_map(r,r,r,r),
      re_update_object(r,r,r,r),
      re_update_action_recipe(r,r,r,r),
      re_update_map(r,r,r,r).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% HIGHEST-LEVEL ROBOEARTH INTERFACE: queries including capability matching and
%                                    download of related information
%



% re_download_action_recipe(+StringCommand, +Robot, -Action) is nondet.
%
% Download a recipe that fits the description StringCommand
% (e.g. 'serve a drink'), perform capability matching to identify
% and download missing components.
%
% @param StringCommand Command to be performed, as described in the recipe by its rdfs:label
% @param Robot         Robot instance modeled in SRDL that the action is to be performed on
% @param Action        Action class description (action recipe)
%
re_download_action_recipe(StringCommand, Robot, Action) :-

  re_request_recipe_for(StringCommand, Action),!,
  re_download_components_for_action(Action, Robot).



% re_download_components_for_action(+Action, +Robot)
%
% Find and request components missing on Robot for execution of a recipe Action.
%
% @param Action Action class to be performed
% @param Robot  Robot instance modeled in SRDL that the action is to be performed on
%
re_download_components_for_action(Action, Robot) :-

    print('\nChecking for missing components...\n'),


    findall(MissingComp, missing_comp_for_action(Action, Robot, MissingComp), MissingComponents),


    % % % % % % % % % % % % % % % % % % % % % % % % %
    % download components if something is missing

    ((length(MissingComponents, Length), Length > 0)->

      ( sort(MissingComponents, MissingComponentsUnique),

        print('* Downloading missing components: '), print(MissingComponentsUnique),print('\n')
        %TODO: download model for each missing component and fail if not successful

      ) ; (
        print('* Missing hardware components: none\n')
      )),



    % % % % % % % % % % % % % % % % % % % % % % % % %
    % check that we also have models for all objects
    % that are somehow related to the action

    findall(ObjT, (plan_subevents_recursive(Action,Sub),
                   class_properties(Sub,_,Obj),

                   atom(Obj), % discard literal(...)

                   % read models for both object instances and classes
                  (  owl_subclass_of(Obj,knowrob:'HumanScaleObject'), ObjT=Obj ;
                    (owl_individual_of(Obj, ObjT),
                     owl_subclass_of(ObjT, knowrob:'HumanScaleObject'))),

                  % do not download models for robot components (gripper etc).
                  (\+ srdl2:comp_type_available(Robot, ObjT)) ), ObjTs),

    sort(ObjTs, ObjTsUnique),

    % % % % % % % % % % % % % % % % % % % % % % % % %
    % download a model for those objects for which
    % we do not have one yet
    print('* Missing object models:\n'),

    findall(OwlFile, (member(ObjT, ObjTsUnique),
                      rdf_split_url(_, Cl, ObjT),

                      (\+ rdf_has(_Model, roboearth:providesModelFor, ObjT)),
                      re_download_model_for(ObjT, OwlFile),

                      print(' * Model for '),print(Cl), print('\n'),
                      owl_parse(OwlFile, false, false, true),print('\n')), _OwlFiles)%,


    % % % % % % % % % % % % % % % % % % % % % % % % %
    % send downloaded models to vision system

    %jpl_list_to_array(OwlFiles, _OWLFileArray),
    %jpl_call('roboearth.wp5.REClients', 'updateObjectModels', [OWLFileArray], _)
    .



% re_generate_cpl_plan(+Plan, -CPLplan) is nondet.
%
% Generate CPL plan from a recipe using the KnowRob PlanExporter
%
% @param Plan     Action recipe class definition
% @param CPLplan  Generated CPL plan
%
re_generate_cpl_plan(Plan, CPLplan) :-
  jpl_new('edu.tum.cs.ias.knowrob.PlanExporter', [], PlanExporter),
  jpl_call(PlanExporter, 'exportPlanToCPL', [Plan], CPLplan).






% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% HIGH-LEVEL ROBOEARTH INTERFACE: Retrieval based on 'semantic queries' via SeRQL
%


% re_download_model_for(ObjectClass, OwlFile)
%
% Download all models for an object class, save the related files and return
% the filename of the OWL description
%
% @param ObjectClass  OWL class of the object for which a model is needed
% @param OwlFile      File name of the OWL file with the object description
%
re_download_model_for(ObjectClass, OwlFile) :-

  re_request_model_for(ObjectClass, ModelURL),
  jpl_call('roboearth.wp5.REClients', 'downloadModelFrom', [ModelURL], OwlFile).



% re_request_model_for(ObjectClass, ModelURL)
%
% Search for object models for a specific type of objects and return the URLs
% of the single models for each result
%
% @param ObjectClass  OWL class of the object for which a model is needed
% @param ModelURL     URL of a suitable model (json description) in the RoboEarth KB
%
re_request_model_for(ObjectClass, ModelURL) :-

  rdf_global_id(ObjGlobLoc, ObjectClass),
  term_to_atom(ObjGlobLoc, ObjGlobLoc2),

  jpl_call('roboearth.wp5.REClients', 'requestModelFor', [ObjGlobLoc2], Array),
  jpl_array_to_list(Array, List),
  List \= [],
  member(ModelURL, List).



% re_request_recipe_for(Command, RecipeClass)
%
% Search for action recipe with the appropriate command string
%
% @param Command      Command to be performed, as described in the recipe by its rdfs:label
% @param RecipeClass  Action class of the recipe as downloaded from RoboEarth
%
re_request_recipe_for(Command, RecipeClass) :-

  print('\nDownloading recipe for command:'),print(Command), print('\n'),

  jpl_call('roboearth.wp5.REClients', 'requestActionRecipeFor', [Command], Array),
  jpl_array_to_list(Array, List),
  member(RecipeURL, List),

  jpl_call('roboearth.wp5.REClients', 'downloadRecipeFrom', [RecipeURL], OwlFile),
  owl_parse(OwlFile, false, false, true), print('\n'),

  % select and return the recipe class
  atomic_list_concat(['*', Command, '*'], Like),
  rdf(RecipeClass, rdfs:label, literal(like(Like), _)).




% re_request_map_for(+RoomNumber, -EnvURL)
%
% Search for and download suitable environment maps and distribute them
% locally on the robot (start map server, send to vslam, init world model
% for semantic maps)
%
% @tbd adapt interface to pass a list of atoms to requestEnvironmentMapFor
%      from which the query is constructed like:
%
%
% @param Query   Spec of the room for which an environment model is needed. Description
%                from detailed to coarse, e.g.
%                [['kr:roomNumber', 3001], ['kr:floorNumber', '3'], ['kr:streetNumber', '45'], ['rdfs:label', 'Karlstrasse']]
% @param EnvURL  URL of a suitable environment model (json description) in the RoboEarth KB
%
re_request_map_for(Query, EnvURL) :-


  % % % % % % % % % % % % % % % % % % % % % % % % %
  % search for environment map
  lists_to_arrays(Query, QueryArray),
  jpl_call('roboearth.wp5.REClients', 'requestEnvironmentMapFor', [QueryArray], Array),
  jpl_array_to_list(Array, List),
  member(EnvURL, List),


  % % % % % % % % % % % % % % % % % % % % % % % % %
  % download and parse OWL file
  jpl_new('java.util.ArrayList', [], FileNames),
  jpl_call('roboearth.wp5.REClients', 'downloadEnvironmentMapFrom', [EnvURL, 'amigo.amigo_robot1', FileNames], OwlFile),
  owl_parse(OwlFile, false, false, true),


  % % % % % % % % % % % % % % % % % % % % % % % % %
  % check that we also have models for all objects
  % in the map

  findall(ObjT, ( owl_individual_of(Map, knowrob:'SemanticEnvironmentMap'),
                  owl_has(Obj, knowrob:describedInMap, Map),
                  (\+ owl_individual_of(Obj, knowrob:'RoomInAConstruction')),
                  rdf_has(Obj, rdf:type, ObjT) ), ObjTs),

  sort(ObjTs, ObjTsUnique),


  % % % % % % % % % % % % % % % % % % % % % % % % %
  % download a model for those objects for which
  % we do not have one yet
  print('* Missing object models:\n'),

  findall(ModelOwlFile, (member(ObjT, ObjTsUnique),
                    rdf_split_url(_, Cl, ObjT),

                    (\+ rdf_has(_Model, roboearth:providesModelFor, ObjT)),
                    re_download_model_for(ObjT, ModelOwlFile),

                    print(' * Model for '),print(Cl), print('\n'),
                    owl_parse(ModelOwlFile, false, false, true),print('\n')), OwlFiles),

  % % % % % % % % % % % % % % % % % % % % % % % % %
  % send downloaded models to vision system

  ((OwlFiles\=[]) -> (
    jpl_list_to_array(OwlFiles, OWLFileArray),
    jpl_call('roboearth.wp5.REClients', 'updateObjectModels', [OWLFileArray], _)); true),


  % % % % % % % % % % % % % % % % % % % % % % % % %
  % init world model if semantic map has been downloaded
  findall(Obj, (owl_individual_of(Map, knowrob:'SemanticEnvironmentMap'),
                owl_has(Obj, knowrob:describedInMap, Map),
                (\+ owl_individual_of(Obj, knowrob:'RoomInAConstruction')),

                current_object_pose(Obj, Pose),
                rdf_has(Obj, rdf:type, Type),
                Type \= 'http://www.w3.org/2002/07/owl#NamedIndividual',

                add_world_model_object(Obj, Type, Pose)), _Objs),


  % % % % % % % % % % % % % % % % % % % % % % % % %
  % if vSLAM map: send to re_vslam node
  ((jpl_call(FileNames, 'toArray', [], FileArray),
    jpl_array_to_list(FileArray, FileList),
    member(File, FileList),
    atom_concat(_, 'vslam_map.tar.gz', File),
    jpl_call('roboearth.wp5.REClients', 'updateVslamMaps', [EnvURL], _)
  ) -> true ; true),


  % % % % % % % % % % % % % % % % % % % % % % % % %
  % start map server if yaml file has been downloaded

  ((jpl_call(FileNames, 'toArray', [], FileArray),
    jpl_array_to_list(FileArray, FileList),
    member(File, FileList),
    atom_concat(_, 'yaml', File),
%     print('start map server: '), print(File),
    re_start_map_server(File, _)
  ) -> true ; true).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% LOW-LEVEL ROBOEARTH INTERFACE: direct retrieval based on the name of an item
%
%

re_request_object(ObjectID, File) :-
  jpl_call('roboearth.wp5.REClients', 'requestObject', [ObjectID], File).

re_request_action_recipe(ID, File) :-
  jpl_call('roboearth.wp5.REClients', 'requestActionRecipe', [ID], File).

re_request_map(MapID, File) :-
  jpl_call('roboearth.wp5.REClients', 'requestMap', [MapID], File).


re_submit_object(Object, ID, Cls, Description) :-
  atomic_list_concat([ID, '.owl'], ObjectOWL),
  export_object(Object, ObjectOWL),
  absolute_file_name(ObjectOWL, ObjectOWLabs),
  jpl_call('roboearth.wp5.REClients', 'submitObject', [ObjectOWLabs,ID,Cls,Description], @true).

re_submit_action_recipe(ActionRecipe, ID, Cls, Description) :-
  atomic_list_concat([ID, '.owl'], RecipeOWL),
  export_action(ActionRecipe, RecipeOWL),
  absolute_file_name(RecipeOWL, RecipeOWLabs),
  jpl_call('roboearth.wp5.REClients', 'submitActionRecipe', [RecipeOWLabs,ID,Cls,Description], @true).

re_submit_map(Map, ID, Cls, Description) :-
  atomic_list_concat([ID, '.owl'], MapOWL),
  export_map(Map, MapOWL),
  absolute_file_name(MapOWL, MapOWLabs),
  jpl_call('roboearth.wp5.REClients', 'submitMap', [MapOWLabs,ID,Cls,Description], @true).


re_update_object(Object, ID, Description) :-
  atomic_list_concat([ID, '.owl'], ObjectOWL),
  export_object_class(Object, ObjectOWL),
  absolute_file_name(ObjectOWL, ObjectOWLabs),
  jpl_call('roboearth.wp5.REClients', 'updateObjectOWL', [ObjectOWLabs,ID,Description], @true).

re_update_action_recipe(ActionRecipe, ID, Description) :-
  atomic_list_concat([ID, '.owl'], RecipeOWL),
  export_action(ActionRecipe, RecipeOWL),
  absolute_file_name(RecipeOWL, RecipeOWLabs),
  jpl_call('roboearth.wp5.REClients', 'updateActionRecipe', [RecipeOWLabs,ID,Description], @true).

re_update_map(Map, ID, Description) :-
  atomic_list_concat([ID, '.owl'], MapOWL),
  export_map(Map, MapOWL),
  absolute_file_name(MapOWL, MapOWLabs),
  jpl_call('roboearth.wp5.REClients', 'updateMap', [MapOWLabs,ID,Description], @true).






% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Utilities: modify environment model
%
%



% re_set_opening_radius(+Door, +Radius) is det.
%
% Convenience predicate: set the opening radius of a door
% e.g. 'http://www.roboearth.org/kb/cabinet_model.owl#IkeaExpeditInsertDoor'
%
% @param Door   Instance of Door
% @param Radius Opening radius of that door
%
re_set_opening_radius(Door, Radius) :-
  rdf_assert(Door, rdfs:subClassOf, '__Description1234'),
  rdf_assert('__Description1234', rdf:type, owl:'Restriction'),
  rdf_assert('__Description1234', owl:onProperty, 'http://www.roboearth.org/kb/cabinet_model.owl#openingRadius'),
  rdf_assert('__Description1234', owl:hasValue, literal(type(xsd:float, Radius))).


% re_update_opening_radius(+Door, +Radius) is det.
%
% Convenience predicate: update the opening radius of a door
% e.g. 'http://www.roboearth.org/kb/cabinet_model.owl#IkeaExpeditInsertDoor'
%
% @param Door   Instance of Door
% @param Radius Opening radius of that door
re_update_opening_radius(Door, Radius) :-
  owl_has(Door, rdfs:subClassOf, Restr),
  owl_has(Restr, rdf:type, owl:'Restriction'),
  owl_has(Restr, owl:onProperty, 'http://www.roboearth.org/kb/cabinet_model.owl#openingRadius'),!,
  rdf_retractall(Restr, owl:hasValue, _),
  rdf_assert(Restr, owl:hasValue, literal(type(xsd:float, Radius))).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Utilities: interface to ROS components
%
%


% re_start_map_server(+YamlFile, -ProcHandle) is nondet.
%
% Start a map_server using the provided YamlFile describing a map
%
% @param YamlFile   YAML description of the environment map
% @param ProcHandle Process handle, useful for killing
% @see http://www.ros.org/wiki/map_server
%
:- assert(map_server(fail)).
re_start_map_server(YamlFile, ProcHandle) :-
  jpl_list_to_array(['__name:=map_server', YamlFile], ArgsArray),
  jpl_call('edu.tum.cs.ias.knowrob.utils.ros.RosUtilities', 'rosrun', ['map_server', 'map_server', ArgsArray], ProcHandle),
  retract(map_server(fail)),
  assert(map_server(ProcHandle)),!.


% re_kill_map_server(+ProcHandle) is nondet.
%
% Kill a map server based on the process handle
%
% @param ProcHandle Process handle as returned by re_start_map_server
%
re_kill_map_server(ProcHandle) :-
  map_server(ProcHandle),
  ProcHandle \= fail,
  jpl_call('edu.tum.cs.ias.knowrob.utils.ros.RosUtilities', 'kill', [ProcHandle], _).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% deprecated: CoP interface methods
%


%% re_request_cop_model_for_objclass(+ObjectClass) is nondet.
%
% Request a CoP model from the RoboEarth data base
%
% The result is automatically sent to the Cop new signatures topic,
% and since CoP reports all new models to KnowRob anyways, we do
% not need to take any further actions here.
%
% @param ObjectClass Object class a model is to be downloaded for
%
re_request_cop_model_for_objclass(ObjectClass) :-
  jpl_call('roboearth.wp5.REClients', 'requestCopModel', [ObjectClass], _).


%% re_export_cop_model_by_id(+ObjectID) is nondet.
%
% Upload a CoP model identified by its ID to the RoboEarth data base
%
% @param ObjectID Id of the model to be uploaded
%
re_export_cop_model_by_id(ObjectID) :-
  jpl_call('roboearth.wp5.REClients', 'exportCopModel', [ObjectID], _).

