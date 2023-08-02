#!/usr/bin/env python3

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
from acxon_module import *
from prolog_module import *
from rosprolog_client import PrologException, Prolog


if __name__ == '__main__':
    rospy.init_node('acxon_test')
    roslib.load_manifest('rosprolog')
    
    # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    t_locality = [1.0, 5.0]
    specificity = 3 # from 1 to 3
    pairs_to_query = [["dul:'Plan'", "dul:'Plan'"]]
    constrained_ontological_scope = ["dul:'Quality'", "dul:'Event'"] # classes to constrain the scope of the narrative
    narratives_file = rospack.get_path('explanatory_narratives_cra') + "/txt/acxon_based/generated_c_narratives_with_specificity_" + str(specificity) + ".txt"

    tuples_dict, pairs_id_to_pairs_to_compare_dict = retrieve_narrative_tuples_(client_rosprolog, pairs_to_query, t_locality, constrained_ontological_scope, specificity)

    
    #f = open(narratives_file, "w")

    for pair_to_compare_id, tuples_of_the_pair in tuples_dict.items():
      introductory_text = "\n\n·····C-Narrative for: " + pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][0] \
        + " and " + pairs_id_to_pairs_to_compare_dict[pair_to_compare_id][1] + "\n"
      narrative = construct_narrative(client_rosprolog, pairs_id_to_pairs_to_compare_dict[pair_to_compare_id], \
                                      tuples_of_the_pair)
      
      print(introductory_text)
      print(narrative)
      
      #f.write(introductory_text)
      #f.write(narrative)

    #f.close()

    print("[acxon_test.py] Narratives have been properly generated, check the 'txt'.")