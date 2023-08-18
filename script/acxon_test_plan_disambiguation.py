#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

"""
What is this code?
  - acxon test for ontology-based narratives. 
  You can expect to find here an example of use of the algorithm for explanatory ontology-based narratives in the 
  collaborative robotics and adaptation domain. Make sure that you have properly selected the algorithm parameters
  (e.g., the temporal locality (t_locality), the specificity, etc.). You can modify them below.

"""

import re
import rospy
import roslib
import rospkg
from acxon.acxon_module import *
from prolog.prolog_module import *
from rosprolog_client import PrologException, Prolog
from know_cra.rosplan_cra_module import ROSPlanCRA

if __name__ == '__main__':
    rospy.init_node('acxon_test_plan_disambiguation')
    roslib.load_manifest('rosprolog')
    
    # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    human_name_planning_domain = 'the_manager'
    t_locality = [1.0, 5.0]
    specificity = 3 # from 1 to 3
    
    # pairs of objects to c-narrate
    pairs_of_objects_to_bring = [["ocra_home:'Drink'", "ocra_home:'Drink'"]]
    constrained_ontological_scope = ["dul:'Quality'", "dul:'Role'", "dul:'Place'"] # classes to constrain the scope of the narrative
    narratives_file = rospack.get_path('explanatory_narratives_cra') + "/txt/acxon_based/generated_c_narratives_plan_disambiguation_with_specificity_" + str(specificity) + ".txt"

    tuples_dict, pairs_id_to_pairs_to_compare_dict = retrieve_narrative_tuples_(client_rosprolog, pairs_of_objects_to_bring, t_locality, constrained_ontological_scope, specificity)

    
    f = open(narratives_file, "w")

    for pair_to_compare_id, tuples_of_the_pair in tuples_dict.items():
      introductory_text_objects_start = "\n\n·····The robot could bring you the " 
      introductory_text_objects_pair = \
        extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]) \
        + " or the " + \
        extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]) 
      introductory_text_objects_end = "\n\n"
      
      introductory_text_objects = introductory_text_objects_start + introductory_text_objects_pair + introductory_text_objects_end

      objects_c_narrative = construct_narrative(client_rosprolog, pairs_id_to_pairs_to_compare_dict[pair_to_compare_id], \
                                      tuples_of_the_pair)
      
      print(introductory_text_objects)
      print(objects_c_narrative)
      
      

      # Generate a plan for each of the instances in each pair
      pair_of_objects_to_bring_with_uri = pairs_id_to_pairs_to_compare_dict[pair_to_compare_id]
      pair_of_objects_to_bring_name = list()
      for i in range(0, len(pair_of_objects_to_bring_with_uri)): 
        rpcra = ROSPlanCRA()

        object_ = extract_individual_from_kb_answer(pair_of_objects_to_bring_with_uri[i])
        pair_of_objects_to_bring_name.append(object_)

        # update the planning knowledge base with with a new goal (the object_ at the human)
        if i == 0: 
          rpcra.rosplan_wrapper_.add_or_remove_single_fact_planning_kb(1, 'object_at', \
                {'p':object_, 'l':human_name_planning_domain}) # add goal
        else:
          previous_object_ = pair_of_objects_to_bring_name[0]
          rpcra.rosplan_wrapper_.add_or_remove_single_fact_planning_kb(3, 'object_at', \
                {'p':previous_object_, 'l':human_name_planning_domain}) # remove goal
          rpcra.rosplan_wrapper_.add_or_remove_single_fact_planning_kb(1, 'object_at', \
                {'p':object_, 'l':human_name_planning_domain}) # add goal

        # Generate and parse plan
        rpcra.rosplan_wrapper_.planning_pipeline() 
       
        # Assert plan (meta) knowledge to the ontology KB 
        rpcra.rosplan_wrapper_.construct_plan_dict()
        rpcra.rosplan_wrapper_.plan_dict_['plan_id'] += '_' + object_ # simple way to link objects and plans
        rpcra.rosplan_wrapper_.plan_dict_['task_grounded_parameters_dict'].clear() # PARTIALLY-UGLY solution  
        # TODO: modify 'unitary_plan_predicates_to_ontology_relations_dict_' in know_cra.rosprolog_wrapper_for_rosplan
        ## print(rpcra.rosplan_wrapper_.plan_dict_)

        plan_triples_list = rpcra.rosprolog_wrapper_for_rosplan_cra_.plan_dict_to_triples_list(rpcra.rosplan_wrapper_.plan_dict_)
        """ # for debugging
        for i in range(0, 25):
            print(plan_triples_list[i])
            print("\n")
        """
        ## print(plan_triples_list)

        plan_assertion_query_text = rpcra.rosprolog_wrapper_for_rosplan_cra_.construct_query_text_for_multiple_triples_assertion(plan_triples_list, True)
        ## print(plan_assertion_query_text)
        rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(plan_assertion_query_text)
    

      # pairs of plans to c-narrate
      pairs_of_plans_to_disambiguate = [["dul:'Plan'", "dul:'Plan'"]] 
      constrained_ontological_scope = ["dul:'Quality'", "dul:'Event'"] # classes to constrain the scope of the narrative
      narratives_file = rospack.get_path('explanatory_narratives_cra') + "/txt/acxon_based/generated_c_narratives_plan_disambiguation_with_specificity_" + str(specificity) + ".txt"

      tuples_dict, pairs_id_to_pairs_to_compare_dict = retrieve_narrative_tuples_(client_rosprolog, pairs_of_plans_to_disambiguate, t_locality, constrained_ontological_scope, specificity)

    for pair_to_compare_id, tuples_of_the_pair in tuples_dict.items():
      pair_of_plans_to_c_narrate_name = list()
      for i in range(0, len(pair_of_objects_to_bring_name)):
        if pair_of_objects_to_bring_name[i] in extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]):
          pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]))
        elif pair_of_objects_to_bring_name[i] in extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]): 
          pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]))
        else:
          pass
      
      introductory_text_plans_start = "\n\n·····In order to bring you the " 
      introductory_text_plans_pair = ", the robot will execute " + \
        pair_of_plans_to_c_narrate_name[0] \
        + " or " + \
        pair_of_plans_to_c_narrate_name[0] + ", respectively. \n\n"
      
      introductory_text_plans = introductory_text_plans_start + introductory_text_objects_pair + introductory_text_plans_pair

      plans_c_narrative = construct_narrative(client_rosprolog, pairs_id_to_pairs_to_compare_dict[pair_to_compare_id], \
                                      tuples_of_the_pair)
      
      print(introductory_text_plans)
      print(plans_c_narrative) 

      # TODO : improve the narratives
      # - substitute the name of the plans for something more readable 
      # - modify some properties

      f.write(introductory_text_objects)
      f.write(objects_c_narrative)

      f.write(introductory_text_plans)
      f.write(plans_c_narrative)

    f.close()

    #### print("[acxon_test.py] Narratives have been properly generated, check the 'txt'.")
