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
import time
import rospy
import roslib
import rospkg
from utils.test_module import *
from acxon.acxon_module import *
from prolog.prolog_module import *
from rosprolog_client import PrologException, Prolog
from know_cra.rosplan_cra_module import ROSPlanCRA

if __name__ == '__main__': 
    # TEST variables  
    start = time.time()
    triples_count = 0
    words_count = 0
    test_object = TestClassForExplanatoryNarratives()

    rospy.init_node('acxon_test_plan_disambiguation_with_neem')
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
      
      ## print(introductory_text_objects)
      ## print(objects_c_narrative)

      # extract names of objects to bring to the user  
      pair_of_objects_to_bring_with_uri = pairs_id_to_pairs_to_compare_dict[pair_to_compare_id]
      pair_of_objects_to_bring_name = list()
      for i in range(0, len(pair_of_objects_to_bring_with_uri)): 
        object_ = extract_individual_from_kb_answer(pair_of_objects_to_bring_with_uri[i])
        pair_of_objects_to_bring_name.append(object_)

      triples_count = triples_count + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]]) + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]])
      words_count = words_count + len(objects_c_narrative.split())


    # pairs of plans to c-narrate
    pairs_of_plans_to_disambiguate = [["dul:'Plan'", "dul:'Plan'"]] 
    constrained_ontological_scope = ["dul:'Quality'", "dul:'Event'"] # classes to constrain the scope of the narrative
    ## constrained_ontological_scope = [] # no constrain at all, the narrative will use all the stored knowledge 
    narratives_file = rospack.get_path('explanatory_narratives_cra') + "/txt/acxon_based/generated_c_narratives_plan_disambiguation_with_specificity_" + str(specificity) + ".txt"

    tuples_dict, pairs_id_to_pairs_to_compare_dict = retrieve_narrative_tuples_(client_rosprolog, pairs_of_plans_to_disambiguate, t_locality, constrained_ontological_scope, specificity)

    for pair_to_compare_id, tuples_of_the_pair in tuples_dict.items():
      pair_of_plans_to_c_narrate_name = list()
      if (pair_of_objects_to_bring_name):
        for i in range(0, len(pair_of_objects_to_bring_name)):
          if pair_of_objects_to_bring_name[i] in extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]):
            pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]))
          elif pair_of_objects_to_bring_name[i] in extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]): 
            pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]))
          else:
            pass
      else: 
        pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]))
        pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]))

      # narrative construction
      introductory_text_plans_start = "\n\n·····In order to bring you the " 
      introductory_text_plans_pair = ", the robot will execute " + \
        pair_of_plans_to_c_narrate_name[0] \
        + " or " + \
        pair_of_plans_to_c_narrate_name[1] + ", respectively. \n\n"
      
      introductory_text_plans = introductory_text_plans_start + introductory_text_objects_pair + introductory_text_plans_pair

      plans_c_narrative = construct_narrative(client_rosprolog, pairs_id_to_pairs_to_compare_dict[pair_to_compare_id], \
                                      tuples_of_the_pair)
      
      triples_count = triples_count + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]]) + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]])
      words_count = words_count + len(plans_c_narrative.split())
      
      ## print(introductory_text_plans)
      ## print(plans_c_narrative) 

      # TODO : improve the narratives
      # - substitute the name of the plans for something more readable [DONE]
      # - modify some properties (e.g., far from / near to -> is far from / is close to, etc.) [DONE - acxon_module.py]
      # - substitute the 'has data value' when it appears [PARTIALLY DONE - with previous (propperties modficiation)]

      # Combine all the narratives
      combined_c_narrative = introductory_text_objects + objects_c_narrative + introductory_text_plans + plans_c_narrative
      ## print("\nC-Narrative")
      ## print(combined_c_narrative)

      # Modify the narrative to make it more appealing
      combined_c_narrative_mod = combined_c_narrative.replace(pair_of_plans_to_c_narrate_name[0], "Plan_A")
      combined_c_narrative_mod = combined_c_narrative_mod.replace(pair_of_plans_to_c_narrate_name[1], "Plan_B")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has worse quality value", "has a higher value")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has better quality value", "has a lower value")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has equivalent quality value than", "has the same value as")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("has role", "is classified as")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("is role of", "classifies")
      combined_c_narrative_mod = combined_c_narrative_mod.replace("defines task", "includes task")

      ## print("\nC-Narrative modified")
      ## print(combined_c_narrative_mod)

      f.write(combined_c_narrative_mod)

    f.close()

    print("[acxon_test_plan_disambiguation_with_neem.py] Narratives have been properly generated, check the 'txt'.")
    
    end = time.time()
    
    print("\n ·· TEST ·····\n Elapsed time: ", end - start, " seconds")
    print(" Memory usage (peak): ", test_object.get_memory_usage()['vmpeak'], " MB") # 'vmpeak' 'vmsize' 'vmlck' 'vmpin' 'vmhwm' 'vmrss' 'vmdata' 'vmstk' 'vmexe' 'vmlib' 'vmpte' 'vmswap'
    print(" Number of triples: ", triples_count) 
    print(" Narrative number of words (aprox.): ", words_count) # TODO first modify (to count the proper words) it and then add the introductions
    print("\n")