#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

"""
What is this code?
  - acxon fast test for ontology-based narratives. MODIFIED TO SPEED UP THE CONSTRUCTION
  You can expect to find here an example of use of the algorithm for explanatory ontology-based narratives in the 
  collaborative robotics and adaptation domain. Make sure that you have properly selected the algorithm parameters
  (e.g., the temporal locality (t_locality), the specificity, etc.). You can modify them below.

"""

import re
import csv
import time
import rospy
import roslib
import rospkg
from utils.test_module import *
from acxon.acxon_fast_module import *
from prolog.prolog_module import *
from rosprolog_client import PrologException, Prolog
from know_cra.rosplan_cra_module import ROSPlanCRA

if __name__ == '__main__': 
    rospy.init_node('acxon_fast_test_plan_disambiguation_with_neem')
    roslib.load_manifest('rosprolog')

    # TEST variables  
    start = time.time()
    triples_count = 0
    words_count = 0
    test_object = TestClassForExplanatoryNarratives()

    if (rospy.has_param('~specificity_level')):
      specificity = rospy.get_param('~specificity_level')
    else:
      rospy.loginfo(rospy.get_name() + ": ROS parameter cannot be read.")
      specificity = 3
    
    if (rospy.has_param('~domain_name')):
      planning_domain = rospy.get_param('~domain_name')
    else:
      rospy.loginfo(rospy.get_name() + ": ROS parameter cannot be read.")
      planning_domain = "unknown"
    
    if (rospy.has_param('~problem_name')):
      planning_problem = rospy.get_param('~problem_name')
    else:
      rospy.loginfo(rospy.get_name() + ": ROS parameter cannot be read.")
      planning_problem = "unknown"
    
    # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    human_name_planning_domain = 'the_manager'
    t_locality = [1.0, 5.0]
    narratives_file = rospack.get_path('explanatory_narratives_cra') + \
      "/txt/acxon_fast_based/test/"+planning_domain+"/generated_c_narratives_plan_comparison_with_specificity_" + str(specificity) + "_problem_" + planning_problem + ".txt"
    evaluation_results_file = rospack.get_path('explanatory_narratives_cra') + \
      "/csv/acxon_fast_based/test/" + planning_domain + "/problem_" + planning_problem + "_with_specificity_" + str(specificity) + ".csv"
    
    # pairs of objects to c-narrate
    pairs_of_objects_to_bring = [["ocra_home:'Drink'", "ocra_home:'Drink'"]]
    constrained_ontological_scope = ["dul:'Quality'", "dul:'Role'", "dul:'Place'"] # classes to constrain the scope of the narrative

    tuples_dict, pairs_id_to_pairs_to_compare_dict = retrieve_narrative_tuples_(client_rosprolog, pairs_of_objects_to_bring, t_locality, constrained_ontological_scope, specificity)

    
    f = open(narratives_file, "w")

    # pairs of plans to c-narrate
    classes_to_compare = [["dul:'Plan'", "dul:'Plan'"]] 
    ## constrained_ontological_scope = ["dul:'Quality'", "dul:'Event'"] # classes to constrain the scope of the narrative
    constrained_ontological_scope = [] # no constrain at all, the narrative will use all the stored knowledge 
    narratives_file = rospack.get_path('explanatory_narratives_cra') + "/txt/acxon_fast_based/generated_c_narratives_plan_disambiguation_with_specificity_" + str(specificity) + ".txt"

    tuples_dict, pairs_id_to_pairs_to_compare_dict = retrieve_narrative_tuples_(client_rosprolog, classes_to_compare, t_locality, constrained_ontological_scope, specificity)

    for pair_to_compare_id, tuples_of_the_pair in tuples_dict.items():
      pair_of_plans_to_c_narrate_name = list()
      
      pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]))
      pair_of_plans_to_c_narrate_name.append(extract_individual_from_kb_answer(pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]))

      # narrative construction
      introductory_text_plans = "\n\n·····There are two plans to disambiguate: " + \
        pair_of_plans_to_c_narrate_name[0] \
        + " and " + \
        pair_of_plans_to_c_narrate_name[1] + ". \n\n"

      plans_c_narrative = construct_narrative(client_rosprolog, pairs_id_to_pairs_to_compare_dict[pair_to_compare_id], \
                                      tuples_of_the_pair)
      
      triples_count = triples_count + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0]]) + len(tuples_of_the_pair[pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1]])
      words_count = words_count + len(plans_c_narrative.split())
      
      ## print(introductory_text_plans)
      ## print(plans_c_narrative) 

      # TODO : improve the narratives
      # - substitute the name of the plans for something more readable [DONE]
      # - modify some properties (e.g., far from / near to -> is far from / is close to, etc.) [DONE - acxon_fast_module.py]
      # - substitute the 'has data value' when it appears [PARTIALLY DONE - with previous (propperties modficiation)]

      # Combine all the narratives
      combined_c_narrative = introductory_text_plans + plans_c_narrative
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

    print("[acxon_fast_test_plan_disambiguation_with_neem.py] Narratives have been properly generated, check the 'txt'.")
    
    end = time.time()
    
    # results dictionary
    results_dict = {"Domain": planning_domain, "Problem" : planning_problem, "Specificity" : specificity, "Time" : (end - start), "Memory" : test_object.get_memory_usage()['vmpeak'], "Triples" : triples_count, "Words": words_count}
    

    with open(evaluation_results_file, 'w', newline='') as g:
      fieldnames = list(results_dict.keys())
      writer = csv.DictWriter(g, fieldnames=fieldnames)

      writer.writeheader()
      writer.writerow(results_dict)
    
    print("\n ·· TEST ·····\n Elapsed time: ", results_dict["Time"], " seconds")
    print(" Memory usage (peak): ", results_dict["Memory"], " MB") # 'vmpeak' 'vmsize' 'vmlck' 'vmpin' 'vmhwm' 'vmrss' 'vmdata' 'vmstk' 'vmexe' 'vmlib' 'vmpte' 'vmswap'
    print(" Number of triples: ", results_dict["Triples"]) 
    print(" Narrative number of words (aprox.): ", results_dict["Words"]) 
    print("\n")