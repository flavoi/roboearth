/** <module> comp_re_vision

  This module provides routines to interface the re_vision perception
  system

  Copyright (C) 2010 by Moritz Tenorth

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

:- module(comp_re_vision,
    [
      re_to_knowrob/2,
      re_vision_listener/1%,
%       re_object_pose/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_meta re_to_knowrob(r,r),
            re_object_pose(r,-).

%% re_vision_listener(-Listener)
re_vision_listener(Listener) :-
    jpl_new('org.roboearth.re_ontology.ReVisionROSClient', ['json_prolog'], Listener),
    jpl_call(Listener, 'startObjDetectionsListener', ['/re_world_model/filtered_object_locations'], _).




%% re_create_perception_instance(-Perception) is det.
%
% Create perception instance
%
% @deprecated Please use knowrob_perception:create_perception_instance
%
% re_create_perception_instance(Perception) :-
%   create_perception_instance(['ReVisionPerception'], Perception).



%% re_create_object_instance(+ObjTypes, +ObjID, -Obj) is det.
%
% Create object instance having all the types in ObjTypes
%
% @deprecated Please use knowrob_perception:create_object_instance
%
% re_create_object_instance(ObjTypes, ObjID, Obj) :-
%   create_object_instance(ObjTypes, ObjID, Obj).



%% re_set_object_perception(?A, ?B) is det.
%
% Link the object instance to the perception instance
%
% @deprecated Please use knowrob_perception:set_object_perception
%
% re_set_object_perception(Object, Perception) :-
%   rdf_assert(Perception, knowrob:objectActedOn, Object).


%% re_set_perception_pose(+Perception, +PoseList) is det.
%
% Set the pose of an object perception
%
% @deprecated Please use knowrob_perception:set_perception_pose
%
% re_set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-
%   set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]).



%% re_object_pose(+ObjInstance, +PoseList) is det.
%
% Get the pose of an object based on the latest perception
%
% @deprecated Please use knowrob_perception:current_object_pose
%
% re_object_pose(Obj, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-
%   current_object_pose(Obj, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]).


%% re_to_knowrob(?ReIdentifier, ?KnowrobIdentifier) is det.
%
% This predicate defines a simple mapping to translate from any kind of identifier in re_vision
% (class names, model names, etc) into the corresponding identifiers in KnowRob.
%
% @param ReIdentifier     Atom identifying something in re_vision
% @param KnowrobIdentifier Corresponding atom identifying something in KnowRob
%
re_to_knowrob('chair', K)    :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Chair-PieceOfFurniture',!.
re_to_knowrob('bed', K)      :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Bed-PieceOfFurniture',!.
re_to_knowrob('cabinet', K)  :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Cabinet-PieceOfFurniture',!.
re_to_knowrob('bottle1', K) :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle',!.

