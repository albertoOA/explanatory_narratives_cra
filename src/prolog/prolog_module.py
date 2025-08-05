#!/usr/bin/env python3

"""
What is this code?
  - prolog module for explanatory ontology-based narratives for collaborative robotics and adaptation. 
  You can expect to find here definitions that are useful to query and assert knowledge to a prolog knowledge 
  base that uses the ontology for collaborative robotics and adaptation (OCRA).

What is defined in it?
  - ManualAssertionOfNEEMs: a class to automatically assert knowledge that has previously been manually
  written. This class will be used when users want to ensure a curated knowledge base instead of using knolwedge
  directly asserted by a robot during the execution of its tasks. The code reads a .pl file with manually created
  NEEMs and asserts them to a knowledge base. Note that the KB (e.g., know_cra) shall be already launched.



  - owl_uri_to_label_dict: it is a dictionary mapping the complete owl uris of the ontologies to 
  the tag labels. 

  - app_ontology_label: it is the label used to refer to the application ontology where the instances 
  to explain are formally defined. In our case, it is 'ocra_filling_a_tray'.

"""

from rosprolog_client import PrologException, Prolog


class ManualAssertionOfNEEMs:
    def __init__(self):
        # ROS useful variables
        self.client_rosprolog = Prolog()

    def read_neems_from_file(self, file_name):
        neems = list()
        file1 = open(file_name, 'r')
        file_lines = file1.readlines()
        
        for line in file_lines:
            if 'kb_project' in line:
                neems.append(line.strip()) # strip() removes spaces to the left and right of the string

        return neems

    def multiple_assertion_in_kb(self, assertions_list):
        done = False
        try:
          for assertion in assertions_list:
              query = self.client_rosprolog.query(assertion)
              ## for solution in query.solutions():
                  ## print(solution)
              query.finish()
          
          done = True
        except PrologException:
            print("Prolog query failed") 

        return done

owl_uri_to_label_dict = {"http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#": "ocra", \
                         "http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_home.owl#": "ocra_home", \
                         "http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_cloth.owl": "ocra_cloth", \
                         "http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_common.owl#": "ocra_common", \
                         "http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra_filling_a_tray.owl#": "ocra_filling_a_tray", \
                         "http://www.iri.upc.edu/groups/perception/OCRA/maps/piling_cloth.owl#": "map_piling_cloth", \
                         "http://www.iri.upc.edu/groups/perception/OCRA/maps/bringing_object.owl#": "map_bringing_object", \
                         "http://www.iri.upc.edu/groups/perception/ont/maps/map_generic_cs.owl#": "map_generic_cs", \
                         "http://www.ease-crc.org/ont/SOMA.owl#": "soma", \
                         "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#": "dul", \
                         "http://www.w3.org/1999/02/22-rdf-syntax-ns#": "rdf", \
                         "http://www.w3.org/2000/01/rdf-schema#": "rdfs", \
                         "http://www.w3.org/2002/07/owl#": "owl"
}

semantic_map_namespace_cloth = "map_piling_cloth"