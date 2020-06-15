
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <json_prolog/prolog.h>

using namespace json_prolog;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_json_prolog");

  Prolog pl;

  //PrologBindings query = pl.once("owl_parse('/home/jkabalar/Downloads/knowrob.owl').");
  //std::cout << query<< std::endl;
  PrologQueryProxy bdgs = pl.query("owl_subclass_of(A, knowrob:'FoodOrDrink').");
  
  
  for(PrologQueryProxy::iterator it=bdgs.begin();
      it != bdgs.end(); it++)
  {
    PrologBindings bdg = *it;
    std::cout << "Found solution: " << std::endl;
    std::cout << "A = "<< bdg["A"] << std::endl;
  }
  //bdgs = pl.next_solution("owl_subclass_of(A, ' http://knowrob.org/kb/knowrob.owl#FoodOrDrink').");
  
  return 0;
}
