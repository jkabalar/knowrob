/** <module> owl_export

  This module contains methods for exporting triples into OWL files,
  for instance object definitions, environment maps, or task specifications.

  Copyright (C) 2011 by Moritz Tenorth

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


:- module(owl_export,
    [
      export_object/2,
      export_object_class/2,
      export_map/2,
      export_action/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).

:- rdf_meta export_object(r,r),
      export_object_class(r,r),
      export_map(r,r),
      export_action(r,r).


%% export_object(+Obj, -File)
%
% Export the perception of an object to an OWL file
%
% @param Obj  Object instance to be exported (including the perception instances)
% @param File Filename for the exported OWL file
%
export_object(Obj, File) :-
  read_object_info(Obj, ObjInfos),
  export_to_owl(ObjInfos, File).


%% export_object_class(+Obj, -File)
%
% Export the definition of an object class to an OWL file
%
% @param Obj  Object class to be exported
% @param File Filename for the exported OWL file
%
export_object_class(Obj, File) :-
  read_objclass_info(Obj, ObjInfos),
  export_to_owl(ObjInfos, File).

%% export_map(+Map, -File)
%
% Export the map as the set of all perceptions of objects to an OWL file
%
% @param Map  Map instance to be exported (including the perception instances for all objects)
% @param File Filename for the exported OWL file
%
export_map(Map, File) :-
  read_map_info(Map, MapInfos),
  export_to_owl(MapInfos, File).


%% export_action(+Action, -File)
%
% Export an action specification (TBOX) to an OWL file
%
% @param Action Action specification to be exported
% @param File   Filename for the exported OWL file
%
export_action(Action, File) :-
  read_action_info(Action, ActionInfos),
  export_to_owl(ActionInfos, File).



%% export_to_owl(+Atoms, -File)
%
% Write all information about Atoms to a file using rdf_save_subject
%
% Atoms is a list that is to be generated by read_object_info, read_map_info, or read_action_info
%
% @param Atoms  List of OWL identifiers that are to be exported using rdf_save_subject
% @param File   Filename for the exported OWL file
%
export_to_owl(Atoms, File) :-

  open(File, write, Stream, [encoding('ascii')]),
  rdf_save_header(Stream, []),

  findall(_, (
    member(Atom, Atoms),
    rdf_save_subject(Stream, Atom, _)
  ), _),

  rdf_save_footer(Stream),
  close(Stream).



%% read_object_info(+Inst, -ObjInfosSorted)
%
% Collect information about object perceptions and assemble it into the ObjInfosSorted list.
%
% @param Inst            Object instance that is to be exported
% @param ObjInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_object_info(Inst, ObjInfosSorted) :-

  % read all direct properties
  findall(Prop, (
              owl_has(Inst,P,Prop),
              rdfs_individual_of(P, 'http://www.w3.org/2002/07/owl#ObjectProperty')
          ), Properties),


  % read all direct properties
  findall(PartInfo, (
              rdf_reachable(Inst, 'http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalPartTypes', Part),
              Part \= Inst,
              read_object_info(Part, PartInfo)
          ), PartInfos),

  % TODO: take transitive properties into account, e.g. to export all parts of an object


  % read all perception instances
  findall(Perc, (
              owl_has(Perc, 'http://ias.cs.tum.edu/kb/knowrob.owl#objectActedOn', Inst),
              owl_individual_of(Perc, 'http://ias.cs.tum.edu/kb/knowrob.owl#MentalEvent')
        ), Perceptions),

  % read properties of all perception instances
  findall(PercProp, (
              member(Perc, Perceptions),
              owl_has(Perc,P,PercProp),
              rdfs_individual_of(P, 'http://www.w3.org/2002/07/owl#ObjectProperty')
          ), PercProperties),

  append([Properties,PartInfos,Perceptions,PercProperties], ObjInfos),
  flatten(ObjInfos, ObjInfosFlat),
  sort(ObjInfosFlat, ObjInfosSorted).



%% read_map_info(+Map, -MapInfosSorted)
%
% Collect information about the map and all object perceptions in it and assemble it into the MapInfosSorted list.
%
% @param Map             Map instance that is to be exported
% @param MapInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_map_info(Map, MapInfosSorted) :-

  % read all objects in the map (knowrob:describedInMap) ...
  findall(Obj,   (owl_has(Obj, 'http://ias.cs.tum.edu/kb/knowrob.owl#describedInMap', Map)), RootObjs),

  % ... and their parts (properPhysicalPartTypes or properPhysicalParts)
  findall(Part, (member(Obj, RootObjs),
                  rdf_reachable(Obj, 'http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalPartTypes', Part)), Parts),

  append([RootObjs, Parts], Objs),

  % combine information for each of these objects and object parts
  findall(MapInfo, (member(Obj, Objs),read_object_info(Obj, MapInfo)), MapInfos),
  flatten(MapInfos, MapInfosFlat),
  sort(MapInfosFlat, MapInfosSorted).



%% read_action_info(+Action, -ActionInfosSorted)
%
% Collect information about the action Action and assemble it into the ActionInfosSorted list.
%
% @param Action             Action class that is to be exported
% @param ActionInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_action_info(Action, ActionInfosSorted) :-

  % recursively read all sub-actions of the action
  plan_subevents_recursive(Action, SubEvents),

  % read all properties for each of them
  findall(Value, (member(Act, SubEvents), action_properties(Act, _, Value)), ActionProperties),

  % read everything related to these things by an ObjectProperty
  findall(PropVal, (member(ActProp, ActionProperties),
                    owl_has(ActProp, P, PropVal),
                    rdfs_individual_of(P, 'http://www.w3.org/2002/07/owl#ObjectProperty')), ActionPropProperties),

  append([SubEvents, ActionProperties, ActionPropProperties], ActionInfos),
  flatten(ActionInfos, ActionInfosFlat),
  sort(ActionInfosFlat, ActionInfosSorted).



%% read_objclass_info(+ObjClass, -ObjClassInfosSorted)
%
% Collect information about the object class ObjClass and assemble it into the ObjClassInfosSorted list.
%
% @param ObjClass             Object class that is to be exported
% @param ObjClassInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_objclass_info(ObjClass, ObjClassInfosSorted) :-

  % read all properties for each of them

  class_properties(ObjClass, ObjProperties),
%   findall(Value, (member(Act, SubEvents), action_properties(Act, _, Value)), ObjProperties),

  % read everything related to these things by an ObjectProperty
  findall(PropVal, (member(ObjProp, ObjProperties),
                    owl_has(ObjProp, P, PropVal),
                    rdfs_individual_of(P, 'http://www.w3.org/2002/07/owl#ObjectProperty')), ObjClassPropProperties),

  append([[ObjClass], ObjProperties, ObjClassPropProperties], ObjClassInfos),
  flatten(ObjClassInfos, ObjClassInfosFlat),
  sort(ObjClassInfosFlat, ObjClassInfosSorted).



class_properties(ObjClass, ObjProperties) :-

  findall(Value, (action_properties(ObjClass, _, Value)), DirectObjProperties),
  findall(SubObjProps, (member(SubObj, DirectObjProperties), class_properties(SubObj, SubObjProps)), SubObjProperties),

  append([DirectObjProperties, SubObjProperties], ObjProps),
  flatten(ObjProps, ObjProperties).




