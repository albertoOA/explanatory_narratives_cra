#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

"""
What is this code?
  - axon test for ontology-based narratives. 
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
from axon.axon_module import *
from utils.test_module import *
from prolog.prolog_module import *
from rosprolog_client import PrologException, Prolog
from know_cra.rosplan_cra_module import ROSPlanCRA

if __name__ == '__main__':
    rospy.init_node('axon_test_plan_disambiguation_with_neem')
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
    t_locality = [0.0, 1000.0]
    classes_to_query = ["dul:'Plan'"]
    narratives_file = rospack.get_path('explanatory_narratives_cra') + \
      "/txt/axon_based/test/"+planning_domain+"/generated_narratives_plan_comparison_with_specificity_" + str(specificity) + "_problem_" + planning_problem + ".txt"
    evaluation_results_file = rospack.get_path('explanatory_narratives_cra') + \
      "/csv/axon_based/test/" + planning_domain + "/problem_" + planning_problem + "_with_specificity_" + str(specificity) + ".csv"

    triples_dict = retrieve_narrative_tuples_(client_rosprolog, classes_to_query, t_locality, specificity)

    
    f = open(narratives_file, "w")

    narrative = ""
    for queried_instance, triples in triples_dict.items():
      triples_count = triples_count + len(triples)

      instance_narrative = construct_narrative(client_rosprolog, queried_instance, triples)

      narrative = narrative + instance_narrative
    
    introductory_text = "\n\n·····There are two plans to disambiguate: " + \
        list(triples_dict.keys())[0] \
        + " and " + \
        list(triples_dict.keys())[1] + ". \n\n" 
    combined_narrative = introductory_text + narrative

    # Modify the narrative to make it more appealing
    combined_narrative_mod = combined_narrative.replace(list(triples_dict.keys())[0], "Plan_A")
    combined_narrative_mod = combined_narrative_mod.replace(list(triples_dict.keys())[1], "Plan_B")
    combined_narrative_mod = combined_narrative_mod.replace("has worse quality value", "has a higher value")
    combined_narrative_mod = combined_narrative_mod.replace("has better quality value", "has a lower value")
    combined_narrative_mod = combined_narrative_mod.replace("has equivalent quality value than", "has the same value as")
    combined_narrative_mod = combined_narrative_mod.replace("has role", "is classified as")
    combined_narrative_mod = combined_narrative_mod.replace("is role of", "classifies")
    combined_narrative_mod = combined_narrative_mod.replace("defines task", "includes task")


    f.write(combined_narrative_mod)

    words_count = len(combined_narrative_mod.split())

    f.close()

    print("[axon_test.py] Narratives have been properly generated, check the 'txt'.")

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