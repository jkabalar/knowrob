/*

@author Julia Kabalar

*/


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(failure_ontology).

% parse OWL files, register name spaces
:- owl_parse('package://failure_ontology/owl/failure_geom.owl').
