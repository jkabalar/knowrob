cmake_minimum_required(VERSION 2.8.3)
project(failure_ontology)
find_package(catkin REQUIRED COMPONENTS rosprolog)

catkin_package(CATKIN_DEPENDS rosprolog)
catkin_add_plunit(
   knowrob/knowrob)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
