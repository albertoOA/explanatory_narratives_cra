#!/usr/bin/env python3

"""
What is this code?
  - test module for explanatory ontology-based narratives for collaborative robotics and adaptation. 
  You can expect to find here some util tools for testing explanatory narratives (e.g. test
  tools and functions).

What is defined in it?
  - TestClassForExplanatoryNarratives: a class to perfrom tests for explanatory narratives (e.g. memory
  usage test)


"""


class TestClassForExplanatoryNarratives:
    def __init__(self):
        # Variables
        self.memory_usage_ = 0

    def get_memory_usage(self):
     """Parses /proc/self/status to extract relevant memory figures.
 
     Returns
     -------
     memuse : dict
         A dict from str (name of memory figure) to Float (size in MB)
     """
     memuse = {}
     with open("/proc/self/status") as status:
         # This is not very portable, only works in Unix-like systems      (like Linux).
         for line in status:
             parts = line.split()
             if parts[0].startswith("Vm"):
                 key = parts[0][:-1].lower()
                 memuse[key] = float(parts[1])/1024
     return memuse
    