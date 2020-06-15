#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
    query = prolog.query("owl_parse('/home/jkabalar/Downloads/knowrob.owl').")
    query = prolog.query("owl_subclass_of(A, ' http://knowrob.org/kb/knowrob.owl#FoodOrDrink').")
    for solution in query.solutions():
        print 'Found solution. A = %s' % (solution['A'])
       
        query = prolog.next_solution("owl_subclass_of(A, ' http://knowrob.org/kb/knowrob.owl#FoodOrDrink').")
        print 'Found solution. A = %s' % (solution['A'])

    query.finish()
