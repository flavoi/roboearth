%%
%% Copyright (C) 2010 by Moritz Tenorth
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(ias_knowledge_base).
:- register_ros_package(mod_srdl).
:- register_ros_package(comp_spatial).
:- register_ros_package(re_ontology).
:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).

:- use_module(comp_re_vision).
:- use_module(library(knowrob_objects)).
:- use_module(library(knowrob_actions)).
:- use_module(library(knowrob_perception)).
:- use_module(library(owl_export)).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parser:owl_parse('/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/owl/roboearth.owl', false, false, true).
:- rdf_db:rdf_register_ns(roboearth, 'http://www.roboearth.org/kb/roboearth.owl#',     [keep(true)]).


%:- owl_parser:owl_parse('/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/owl/fmi_hospital_room.owl', false, false, true).


% :- owl_parser:owl_parse('/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/owl/object_model.owl', false, false, true).
% :- rdf_db:rdf_register_ns(re_object_model, 'http://www.roboearth.org/kb/object_model.owl#',     [keep(true)]).

% :- owl_parser:owl_parse('/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/owl/joystick_drive.owl', false, false, true).
% :- rdf_db:rdf_register_ns(re_joystick_drive, 'http://www.roboearth.org/kb/joystick_drive.owl#',     [keep(true)]).

% :- owl_parser:owl_parse('/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/owl/environment_map.owl', false, false, true).
% :- rdf_db:rdf_register_ns(re_environment_map, 'http://www.roboearth.org/kb/environment_map.owl#',     [keep(true)]).

% :- owl_parser:owl_parse('/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/owl/set_a_table.owl', false, false, true).
% :- rdf_db:rdf_register_ns(set_a_table, 'http://www.roboearth.org/kb/set_a_table.owl#',     [keep(true)]).

% :- owl_parser:owl_parse('../owl/serve_drink.owl', false, false, true).
:- rdf_db:rdf_register_ns(serve_drink, 'http://www.roboearth.org/kb/serve_drink.owl#',     [keep(true)]).

:- owl_parser:owl_parse('../owl/amigo.owl', false, false, true).
:- rdf_db:rdf_register_ns(amigo, 'http://www.roboearth.org/kb/amigo.owl#',     [keep(true)]).

:- rdf_db:rdf_register_ns(pr2, 'http://ias.cs.tum.edu/kb/PR2.owl#',     [keep(true)]).

%
% :- owl_parser:owl_parse('../owl/cabinet_model.owl', false, false, true).
:- rdf_db:rdf_register_ns(cabinet, 'http://www.roboearth.org/kb/cabinet_model.owl#',     [keep(true)]).

% :- owl_parser:owl_parse('../owl/map_hospital_room.owl', false, false, true).
:- rdf_db:rdf_register_ns(hospital, 'http://www.roboearth.org/kb/map_hospital_room.owl#',     [keep(true)]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start re_vision listener

% :- re_vision_listener(_).



%% action_primitive(+Action, -Primitive) is nondet.
%
% Reads primitives that provide a certain action
%
action_primitive(Action, Primitive) :-

  owl_subclass_of(Action, Sup),
  owl_restriction(Sup,
                  restriction('http://www.roboearth.org/kb/roboearth.owl#providedByMotionPrimitive',
                        has_value(literal(type('http://www.w3.org/2001/XMLSchemastring', Primitive))))),!.
