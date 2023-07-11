#!/usr/bin/env python3

"""
What is this code?
  - axon test for ontology-based narratives. 
  You can expect to find here an example of use of the algorithm for explanatory ontology-based narratives in the 
  collaborative robotics and adaptation domain. Make sure that you have properly selected the algorithm parameters
  (e.g., the temporal locality (t_locality), the specificity, etc.). You can modify them below.

"""

import re
import rospy
import roslib
import rospkg
from axon_module import *
from prolog_module import *
from rosprolog_client import PrologException, Prolog


if __name__ == '__main__':
    rospy.init_node('axon_test')
    roslib.load_manifest('rosprolog')
    
    # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    t_locality = [0.0, 1000.0]
    specificity = 1 # from 1 to 3
    classes_to_query = ["Collaboration", "PlanAdaptation"]
    narratives_file = rospack.get_path('explanatory_narratives_cra') + "/txt/axon_based/generated_narratives_with_specificity_" + str(specificity) + ".txt"

    triples_dict = retrieve_narrative_tuples_(client_rosprolog, classes_to_query, t_locality, specificity)

    f = open(narratives_file, "w")

    for queried_instance, triples in triples_dict.items():
      introductory_text = "\n\n·····Explanation for: " + queried_instance + "\n"
      narrative = construct_narrative(client_rosprolog, queried_instance, triples)

      f.write(introductory_text)
      f.write(narrative)

    f.close()

    print("[axon_test.py] Narratives have been properly generated, check the 'txt'.")