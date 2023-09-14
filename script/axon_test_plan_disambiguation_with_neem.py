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
    # TEST variables  
    start = time.time()
    triples_count = 0
    words_count = 0
    test_object = TestClassForExplanatoryNarratives()

    rospy.init_node('axon_test_plan_disambiguation_with_neem')
    roslib.load_manifest('rosprolog')
    
   # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    t_locality = [0.0, 1000.0]
    specificity = 3 # from 1 to 3
    classes_to_query = ["dul:'Plan'"]
    narratives_file = rospack.get_path('explanatory_narratives_cra') + "/txt/axon_based/generated_narratives_plan_disambiguation_with_specificity_" + str(specificity) + ".txt"

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
     
    print("\n ·· TEST ·····\n Elapsed time: ", end - start, " seconds")
    print(" Memory usage (peak): ", test_object.get_memory_usage()['vmpeak'], " MB") # 'vmpeak' 'vmsize' 'vmlck' 'vmpin' 'vmhwm' 'vmrss' 'vmdata' 'vmstk' 'vmexe' 'vmlib' 'vmpte' 'vmswap'
    print(" Number of triples: ", triples_count)
    print(" Narrative number of words (aprox.): ", words_count)
    print("\n")