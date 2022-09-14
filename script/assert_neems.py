#!/usr/bin/env python3

"""
What is this code?
  - automatic assertion of facts for manually generated NEEMs to ensure a curated knowledge. 
  You can expect to find here a code that reads a .pl file with manually created NEEMs and asserts them
  to a knowledge base. Note that the KB (know_cra) shall be launched before this code.

"""

import rospy
import roslib
import rospkg
from rosprolog_client import PrologException, Prolog



def read_neems_from_file(file_name):
    neems = list()
    file1 = open(file_name, 'r')
    file_lines = file1.readlines()
     
    for line in file_lines:
        if 'kb_project' in line:
            neems.append(line.strip()) # strip() removes spaces to the left and right of the string

    return neems

def multiple_assertion_in_kb(client_rosprolog, assertions_list):
    done = False

    for assertion in assertions_list:
        query = client_rosprolog.query(assertion)
        #for solution in query.solutions():
        #    print(solution)
        query.finish()

    done = True 

    return done



if __name__ == '__main__':
    rospy.init_node('neem_assertion_script')
    roslib.load_manifest('rosprolog')
    
    # ROS useful variables
    client_rosprolog = Prolog()
    rospack = rospkg.RosPack()

    # settings variables
    neems_to_assert_file = rospack.get_path('explanatory_narratives_cra') + "/prolog/neem_assertion.pl"

    neems_to_assert = read_neems_from_file(neems_to_assert_file)
    #print(neems_to_assert)

    assertions_done = multiple_assertion_in_kb(client_rosprolog, neems_to_assert)
    if assertions_done:
        print("[assert_neems.py] Neems were properly asserted.")
    else:
        print("[assert_neems.py] Neems' assertion failed.")