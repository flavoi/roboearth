#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('json_prolog')

import rospy
import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_re_ontology')
    prolog = json_prolog.Prolog()

    # read the list of actions for the plan SetATable
    query = prolog.query("plan_subevents('http://www.roboearth.org/kb/set_a_table.owl#SetATable', Actions)")
    for solution in query.solutions():
        print 'Found actions for plan SetATable: Actions = %s' % (solution['Actions'])
    query.finish()

    # read the list of actions for the plan SetATable
    query = prolog.query("plan_subevents('http://www.roboearth.org/kb/set_a_table.owl#SetATable', Actions)")
    for solution in query.solutions():
        print 'Found actions for plan SetATable: Actions = %s' % (solution['Actions'])
    query.finish()


		# read the objectActedOn of all actions
    query = prolog.query("plan_subevents('http://www.roboearth.org/kb/set_a_table.owl#SetATable', Actions), member(Act, Actions), comp_ehow:action_objectActedOn(Act, Obj)")
    for solution in query.solutions():
        print 'Found objects: Act = %s, Obj = %s' % (solution['Act'], solution['Obj'])
    query.finish()


		# read the toLocation of all actions
    query = prolog.query("plan_subevents('http://www.roboearth.org/kb/set_a_table.owl#SetATable', Actions), member(Act, Actions), comp_ehow:action_toLocation(Act, Loc)")
    for solution in query.solutions():
        print 'Found locations: Act = %s, Loc = %s' % (solution['Act'], solution['Loc'])
    query.finish()






