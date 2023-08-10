#!/usr/bin/env python3

"""
What is this code?
  - automatic assertion of facts for manually generated NEEMs to ensure a curated knowledge. 
  You can expect to find here a code that uses a class to read a .pl file with manually created 
  NEEMs and asserts them to a knowledge base. Note that the KB (know_cra) shall be launched before this code.

"""

import rospy
import roslib
import rospkg

from prolog.prolog_module import ManualAssertionOfNEEMs



if __name__ == '__main__':
    rospy.init_node('neem_assertion_script')
    roslib.load_manifest('rosprolog')
    
    # ROS useful variables
    maoneems_ = ManualAssertionOfNEEMs()
    rospack = rospkg.RosPack()

    # settings variables
    neems_to_assert_file = rospack.get_path('explanatory_narratives_cra') + "/prolog/neem_assertion_bringing_drink.pl"

    neems_to_assert = maoneems_.read_neems_from_file(neems_to_assert_file)
    ## print(neems_to_assert)

    assertions_done = maoneems_.multiple_assertion_in_kb(neems_to_assert)
    if assertions_done:
        print("[assert_neems.py] NEEMs were properly asserted.")
    else:
        print("[assert_neems.py] NEEMs' assertion failed.")