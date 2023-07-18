#!/usr/bin/env python3

"""
What is this code?
  - acxon module for ontology-based explanations. 
  You can expect to find here all the implemented methods that are used in the ontology-based explanations algorithm (OX).
  For the time being, only the methods to get the tuples from the knowledge based are implemented in this code. We will 
  include the rest of the methods about constructing the textual explanation using the tuples. 

What is defined in it?
  - Two main routines of the OX algorithm: 'retrieve_narrative_tuples' and 'construct_narrative'. Other methods are also
  defined and used to build the two main ones. 


What information could one find useful when using this code?
  - Note that if a query returns 'false' the generator 'query.solutions()' seems to be empty, the code would work but the 
  explanation would be empty.

  - Tuples in this work are lists of six elements, the first trhee are the usual triple elements (subject, property and
  object), the next two denote the time interval in which the triple holds and the last one indicates whether or not the 
  triple's relation should be negative (e.g., 'not has participant')

  - It is good if you write your ontological classes, relationships and individuals withouth spaces and with every word 
  starting character in uppercase (e.g., 'PlanAdapatation', 'hasParticiant'). In order to enhance the readability of the 
  explanations, we split those names using the uppercase characters, and we replace the upper cases in the relatiohnships. 

  - Note that we use query the knowledge base using time intervals, which adds some complexity to the process. When the
  interval is bound (you write a value), the query will only return true if both intervals are the ones you provided. 
  Hence, most of the queries would return false. That is why we have added some logic to our queries to get all the 
  answers that either are included in the bound interval or include the bound interval: 
    (T1>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "+str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2)
  For instance, if we want all the collaborations that exist in the interval 15.0-30.0, we will get a collaboration
  that lasted from 20.0 to 25.0 and also others that lasted from 10.0 to 20.0 or from 28.0 to 40.0. In summary, all
  the instances that were true in that interval. 
    -> We could decide to be more strict and only consider instances whose interval is inside of the provided one.

"""

import re
import ast
from prolog_module import *
from rosprolog_client import PrologException, Prolog


def retrieve_narrative_tuples_(client_rosprolog, ontological_entities_pairs, t_locality, constrained_ontological_scope, specificity):
    tuples = dict() # k: pair_id, v: list of tuples
    pairs_id_to_pairs_to_compare_dict = dict() # k: pair_id, v: pairs

    ont_property_inverse_dict = get_ontology_property_and_inverse_dict(client_rosprolog)

    for entities_pair in ontological_entities_pairs:
        # extract the pairs of instances of the target classes
        instances_pairs_dict = get_pairs_of_instances_of_target_classes(client_rosprolog, entities_pair, t_locality) 

        
        #for assertion_type, instances_list in class_instances.items():
        pairs_list = instances_pairs_dict["pairs"]
        time_intervals_list = instances_pairs_dict["intervals"]
        
        for i in range(0, len(pairs_list)):
            # initialize the dictionary key
            pair_id = "pair_" + str(i)
            tuples[pair_id] = list()
            pairs_id_to_pairs_to_compare_dict[pair_id] = pairs_list[i]

            if (specificity < 1 or specificity > 3):
                print("Error while executing retrieve_narrative_tuples, inccorrect specificity value.")
            else:
                if (specificity >= 1):
                    retrieve_narrative_tuples_specificity_one(client_rosprolog, pair_id, pairs_list[i], time_intervals_list[i], tuples, ont_property_inverse_dict)
                else:
                    pass            
                if (specificity >= 2):
                    retrieve_narrative_tuples_specificity_two(client_rosprolog, pair_id, pairs_list[i], time_intervals_list[i], constrained_ontological_scope, tuples, ont_property_inverse_dict)
                else:
                    pass
                if (specificity == 3):
                    retrieve_narrative_tuples_specificity_three(client_rosprolog, pair_id, pairs_list[i], time_intervals_list[i], constrained_ontological_scope, tuples, ont_property_inverse_dict)
                else:
                    pass

        print(tuples)

    return tuples, pairs_id_to_pairs_to_compare_dict


def construct_narrative(client_rosprolog, target_instance, tuples_in):
    sentence = ''

    ont_property_inverse_dict = get_ontology_property_and_inverse_dict(client_rosprolog)
    
    mod_tuples_cast = cast_tuples(target_instance, tuples_in, ont_property_inverse_dict)
    mod_tuples_clus = cluster_tuples(mod_tuples_cast)
    mod_tuples_ord = order_tuples(target_instance, mod_tuples_clus)
    sentence = group_tuples_in_a_sentence(mod_tuples_ord) 
    sentence = sentence + '\n'
        
    return sentence





# retrieve narrative tuples functions
def retrieve_narrative_tuples_specificity_one(client_rosprolog, pair_id, pair_of_instances, \
                                            instance_time_interval, tuples, ont_property_inverse_dict):
    # note that the variable tuples is modified within this function
    for i in range(0, len(pair_of_instances)):
        q_1 = "kb_call(triple('"+pair_of_instances[0]+"', R, '"+pair_of_instances[1]+"') during [T1, T2])."
        assertion_type_1 = "affirmative"
        
        q_2 = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
            "kb_call(triple(D, owl:'sourceIndividual', '"+pair_of_instances[0]+"') during [T1, T2]), "\
            "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), "\
            "kb_call(triple(D, owl:'targetIndividual', '"+pair_of_instances[1]+"') during [T1, T2])."
        assertion_type_2 = "negative"

        query_aff = client_rosprolog.query(q_1)
        query_neg = client_rosprolog.query(q_2)

        queries_to_kb_dict = {assertion_type_1:query_aff, assertion_type_2:query_neg}

        for assertion_type, query in queries_to_kb_dict.items():
            for solution in query.solutions():
                tr_ = kb_solution_to_tuple_specificity_one(assertion_type, pair_of_instances, solution) 
                ## print(tr_)
                if (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[i][0]) \
                 and extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[i][1])):
                    tr_[3] = ''
                    tr_[4] = ''
                else:
                    pass

                tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
                if tr_ in tuples[pair_id]:
                    ## print("Tuple already in list.")
                    pass
                elif tr_inv_ in tuples[pair_id]:
                    ## print("Inverse tuple already in list.")
                    pass
                else:
                    tuples[pair_id].append(tr_)

            query.finish()


def retrieve_narrative_tuples_specificity_two(client_rosprolog, pair_id, pair_of_instances, \
                                            instance_time_interval, constrained_ontological_scope, \
                                            tuples, ont_property_inverse_dict):
    objects_related_to_instance = dict()
    for i in range(0, len(pair_of_instances)):
        objects_related_to_instance[pair_of_instances[i]] = list()
        
        for ontological_class in constrained_ontological_scope:
            q_1 = "kb_call(triple('"+pair_of_instances[i]+"', R, E) during [T1, T2]), "\
                "kb_call(triple(E, rdf:'type', "+ontological_class+"))."
            assertion_type_1 = "affirmative"

            q_2 = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
                "kb_call(triple(D, owl:'sourceIndividual', '"+pair_of_instances[i]+"') during [T1, T2]), "\
                "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), "\
                "kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2]), "\
                "kb_call(triple(E, rdf:'type', "+ontological_class+"))."
            assertion_type_2 = "negative"

            

            query_aff = client_rosprolog.query(q_1)
            query_neg = client_rosprolog.query(q_2)

            queries_to_kb_dict = {assertion_type_1:query_aff, assertion_type_2:query_neg}

            for assertion_type, query in queries_to_kb_dict.items():
                for solution in query.solutions():
                    tr_ = kb_solution_to_tuple_specificity_greater_than_one(assertion_type, pair_of_instances[i], solution['E'], solution)
                    ## print(tr_)
                    if (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[i][0]) and extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[i][1])):
                        tr_[3] = ''
                        tr_[4] = ''
                    else:
                        pass

                    tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
                    if tr_ in tuples[pair_id]:
                        ## print("Tuple already in list.")
                        pass
                    elif tr_inv_ in tuples[pair_id]:
                        ## print("Inverse tuple already in list.")
                        pass
                    else:
                        tuples[pair_id].append(tr_)
                        objects_related_to_instance[pair_of_instances[i]].append(tr_[2])

                query.finish()
        
    # query for the relationships between the objects of the two instances that are related through the same ontological property
    objects_related_to_instance_a = objects_related_to_instance[pair_of_instances[0]]
    objects_related_to_instance_b = objects_related_to_instance[pair_of_instances[1]]

    for object_a in objects_related_to_instance_a:
        for object_b in objects_related_to_instance_b:
            q_1 = "dif(Oa, E), kb_call(triple("+object_a+", R, "+object_b+") during [T1, T2])"
            assertion_type_1 = "affirmative"

            q_2 = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
                "kb_call(triple(D, owl:'sourceIndividual', "+object_a+") during [T1, T2]), "\
                "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), "\
                "kb_call(triple(D, owl:'targetIndividual', "+object_b+") during [T1, T2])."
            assertion_type_2 = "negative"

            query_aff = client_rosprolog.query(q_1)
            query_neg = client_rosprolog.query(q_2)

            queries_to_kb_dict.clear()
            queries_to_kb_dict = {assertion_type_1:query_aff, assertion_type_2:query_neg}

            for assertion_type, query in queries_to_kb_dict.items():
                for solution in query.solutions():
                    tuple_relation = owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['R'])] \
                        + ":'" + extract_individual_from_kb_answer(solution['R']) + "'"
                    tuple_start = 'start:' + str(solution['T1'])
                    tuple_end = 'end:' + str(solution['T2'])
                    tr_ = construct_tuple_from_its_elements(assertion_type, object_a, object_b, tuple_relation, tuple_start, tuple_end)
                    ## print(tr_)
                    if ((extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[0][0]) and \
                         extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[0][1])) or \
                         (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[1][0]) and \
                         extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[1][1]))):
                        tr_[3] = ''
                        tr_[4] = ''
                    else:
                        pass

                    tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
                    if tr_ in tuples[pair_id]:
                        ## print("Tuple already in list.")
                        pass
                    elif tr_inv_ in tuples[pair_id]:
                        ## print("Inverse tuple already in list.")
                        pass
                    else:
                        tuples[pair_id].append(tr_)

                query.finish()


def retrieve_narrative_tuples_specificity_three(client_rosprolog, pair_id, pair_of_instances, \
                                            instance_time_interval, constrained_ontological_scope, \
                                            tuples, ont_property_inverse_dict):
    objects_related_to_instance = dict()
    for i in range(0, len(pair_of_instances)):
        objects_related_to_instance[pair_of_instances[i]] = list()
        
        for ontological_class in constrained_ontological_scope:
            q_1 = "kb_call(triple('"+pair_of_instances[i]+"', Rx, Ex) during [Tx1, Tx2]), "\
                "kb_call(triple(Ex, rdf:'type', "+ontological_class+")), "\
                "kb_call((triple(Ex, R, E) during [T1, T2], (T1>="+str(instance_time_interval[i][0])+\
                ", T1=<"+str(instance_time_interval[i][1])+"; T2>="+str(instance_time_interval[i][0])+\
                ", T2=<"+str(instance_time_interval[i][1])+"; "+str(instance_time_interval[i][0])+">=T1, "\
                +str(instance_time_interval[1])+"=<T2; T2=:=inf)))."
            assertion_type_1 = "affirmative"

            q_2 = "kb_call(triple('"+pair_of_instances[i]+"', Rx, Ex) during [Tx1, Tx2]), "\
                "kb_call(triple(Ex, rdf:'type', "+ontological_class+")), "\
                "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], "\
                "(T1>="+str(instance_time_interval[i][0])+", T1=<"+str(instance_time_interval[i][1])+"; "\
                "T2>="+str(instance_time_interval[i][0])+", T2=<"+str(instance_time_interval[i][1])+"; "\
                +str(instance_time_interval[i][0])+">=T1, "+str(instance_time_interval[i][1])+"=<T2; T2=:=inf))), "\
                "kb_call(triple(D, owl:'sourceIndividual', Ex) during [T1, T2]), " \
                "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2])."
            assertion_type_2 = "negative"

            q_3 = "kb_call(triple(Dx, Rdx, owl:'NegativePropertyAssertion') during [Tx1, Tx2]), "\
                "kb_call(triple(Dx, owl:'sourceIndividual', '"+pair_of_instances[i]+"') during [Tx1, Tx2]), " \
                "kb_call(triple(Dx, owl:'assertionProperty', Rx) during [Tx1, Tx2]), "\
                "kb_call(triple(Dx, owl:'targetIndividual', Ex) during [Tx1, Tx2]), "\
                "kb_call(triple(Ex, rdf:'type', "+ontological_class+")), "\
                "kb_call((triple(Ex, R, E) during [T1, T2], (T1>="+str(instance_time_interval[i][0])+\
                ", T1=<"+str(instance_time_interval[i][1])+"; T2>="+str(instance_time_interval[i][0])+\
                ", T2=<"+str(instance_time_interval[i][1])+"; "+str(instance_time_interval[i][0])+">=T1, "\
                +str(instance_time_interval[i][1])+"=<T2; T2=:=inf)))."
            assertion_type_3 = "affirmative"

            q_4 = "kb_call(triple(Dx, Rdx, owl:'NegativePropertyAssertion') during [Tx1, Tx2]), "\
                "kb_call(triple(Dx, owl:'sourceIndividual', '"+pair_of_instances[i]+"') during [Tx1, Tx2]), " \
                "kb_call(triple(Dx, owl:'assertionProperty', Rx) during [Tx1, Tx2]), "\
                "kb_call(triple(Dx, owl:'targetIndividual', Ex) during [Tx1, Tx2]), "\
                "kb_call(triple(Ex, rdf:'type', "+ontological_class+")), "\
                "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="\
                +str(instance_time_interval[i][0])+", T1=<"+str(instance_time_interval[i][1])+"; "\
                "T2>="+str(instance_time_interval[i][0])+", T2=<"+str(instance_time_interval[i][1])+"; "\
                +str(instance_time_interval[i][0])+">=T1, "+str(instance_time_interval[i][1])+"=<T2; T2=:=inf))), "\
                "kb_call(triple(D, owl:'sourceIndividual', Ex) during [T1, T2]), "\
                "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), "\
                "kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2])."
            assertion_type_4 = "negative"

            query_aff_1 = client_rosprolog.query(q_1)
            query_neg_2 = client_rosprolog.query(q_2)
            query_aff_3 = client_rosprolog.query(q_3)
            query_neg_4 = client_rosprolog.query(q_4)

            queries_to_kb_dict = {assertion_type_1:[query_aff_1, query_aff_3], \
                                assertion_type_2:[query_neg_2, query_neg_4]}
            
            for assertion_type, queries in queries_to_kb_dict.items():
                for query in queries:
                    for solution in query.solutions():
                        tr_ = kb_solution_to_tuple_specificity_greater_than_one(assertion_type, solution['Ex'], solution['E'], solution)
                        ## print(tr_)
                        if (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[i][0]) and extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[i][1])):
                            tr_[3] = ''
                            tr_[4] = ''
                        else:
                            pass

                        tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
                        ## print(tr_)
                        ## print(tr_inv_)
                        ## print("_-_-_- tr vs inverted\n\n")
                        if tr_ in tuples[pair_id]:
                            ## print("Tuple already in list.")
                            pass
                        elif tr_inv_ in tuples[pair_id]:
                            ## print("Inverse tuple already in list.")
                            pass
                        else:
                            tuples[pair_id].append(tr_)
                            if (tr_[1] != "dul:'hasDataValue'"):
                                objects_related_to_instance[pair_of_instances[i]].append(tr_[2])
                                ## print(tr_)
                                ## print("··\n")
                            else:
                                pass
                            

                    query.finish()

    # query for the relationships between the objects of the two instances that are related through the same ontological property
    objects_related_to_instance_a = objects_related_to_instance[pair_of_instances[0]]
    objects_related_to_instance_b = objects_related_to_instance[pair_of_instances[1]]

    ## print(objects_related_to_instance_a)
    ## print("-\n")
    ## print(objects_related_to_instance_b)
    ## print("--------\n\n")

    for object_a in objects_related_to_instance_a:
        for object_b in objects_related_to_instance_b:
            q_1 = "dif(Oa, E), kb_call(triple("+object_a+", R, "+object_b+") during [T1, T2])"
            assertion_type_1 = "affirmative"

            q_2 = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
                "kb_call(triple(D, owl:'sourceIndividual', "+object_a+") during [T1, T2]), "\
                "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), "\
                "kb_call(triple(D, owl:'targetIndividual', "+object_b+") during [T1, T2])."
            assertion_type_2 = "negative"

            query_aff = client_rosprolog.query(q_1)
            query_neg = client_rosprolog.query(q_2)

            queries_to_kb_dict.clear()
            queries_to_kb_dict = {assertion_type_1:query_aff, assertion_type_2:query_neg}

            for assertion_type, query in queries_to_kb_dict.items():
                for solution in query.solutions():
                    tuple_relation = owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['R'])] \
                        + ":'" + extract_individual_from_kb_answer(solution['R']) + "'"
                    tuple_start = 'start:' + str(solution['T1'])
                    tuple_end = 'end:' + str(solution['T2'])
                    tr_ = construct_tuple_from_its_elements(assertion_type, object_a, object_b, tuple_relation, tuple_start, tuple_end)
                    ## print(tr_)
                    if ((extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[0][0]) and \
                         extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[0][1])) or \
                         (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[1][0]) and \
                         extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[1][1]))):
                        tr_[3] = ''
                        tr_[4] = ''
                    else:
                        pass

                    tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
                    if tr_ in tuples[pair_id]:
                        ## print("Tuple already in list.")
                        pass
                    elif tr_inv_ in tuples[pair_id]:
                        ## print("Inverse tuple already in list.")
                        pass
                    else:
                        tuples[pair_id].append(tr_)

                query.finish()



# construct explanation functions
def cast_tuples(target_instance, tuples_in, ont_prop_dict):
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # unify properties using their inverse: inverse those where the main instance is object 
    # and also make sure that we do not use a property and its inverse (we change the ones 
    # that appear later when going through the tuples)
    casted_tuples = []
    tuples = tuples_in.copy()


    for tuple_ in tuples:
        if target_instance in tuple_[2]:
            ## print("if instance is object")
            if invert_tuple_(tuple_, ont_prop_dict) in casted_tuples: # avoid repetitions when there are more than 2 target instances
                ## print("if inverted in casted")
                pass
            else:
                casted_tuples.append(invert_tuple_(tuple_, ont_prop_dict))
        else:
            ## print("if instance is not object")
            tuple_property = extract_individual_from_tuple_element(tuple_[1])
            concatenated_casted_tuples = []
            casted_tuples_cp = casted_tuples.copy()
            [concatenated_casted_tuples.extend(el) for el in casted_tuples_cp]

            if tuple_property in ont_prop_dict and any(ont_prop_dict[tuple_property] in s for s in concatenated_casted_tuples):
                ## print("if prop in casted")
                if invert_tuple_(tuple_, ont_prop_dict) in casted_tuples: # avoid repetitions e.g. isWorsePlanThan
                    ## print("if inverted in casted")
                    pass
                else:
                    ## print("if inverted not in casted")
                    casted_tuples.append(invert_tuple_(tuple_, ont_prop_dict))
            else:
                ## print("if prop not in casted")
                if tuple_ in casted_tuples: # avoid repetitions when there are more than 2 main instances
                    pass
                else:
                    casted_tuples.append(tuple_)

    
    return casted_tuples


def cluster_tuples(tuples_in):
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # -tuples about a single instance/entity are clustered together
    clustered_tuples_dict = dict()
    tuples = tuples_in.copy()
    
    tuples_cp = tuples.copy()
    cont = 0
    while tuples_cp:
        indices = find_indices_of_related_tuples(tuples_cp[0], tuples_cp) 
        related_tuples = extract_related_tuples(tuples_cp, indices)
        clustered_tuples_dict[cont] = related_tuples
        indices_to_remove = indices.copy()
        indices_to_remove.append(0)
        tuples_cp = [tuples_cp[j] for j in range(0, len(tuples_cp)) if j not in indices_to_remove] 
        cont += 1
    
    return clustered_tuples_dict


def order_tuples(target_instance, tuples_dict_in):
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # -(between sentences) external ordering: much info > less info (except the main instance to explain, it always goes first)
    # -(within a sentence) internal ordering: superclass > attributes
    ordered_tuples = dict()
    ordered_tuples_ext = dict()
    ordered_tuples_int = dict()
    tuples_dict = tuples_dict_in.copy()
    
    # external ordering
    tuples_dict_cp = tuples_dict.copy()
    cont = 0
    while (tuples_dict_cp):
        key_with_max_info = list(tuples_dict_cp.keys())[0]
        for key, value in tuples_dict_cp.items():
            if any(target_instance in tuple_element for tuple_element in value[0]):
                key_with_max_info = key
                break
            else:
                if len(tuples_dict_cp[key_with_max_info]) < len(value):
                    key_with_max_info = key
                else:
                    pass
        
        ordered_tuples_ext[cont] = tuples_dict_cp[key_with_max_info]
        tuples_dict_cp.pop(key_with_max_info)
        cont += 1
        
    # internal ordering
    for key, value in ordered_tuples_ext.items():
        new_value = list()
        
        for v in value:
            if 'type' in v[1]:
                new_value.insert(0, v)
            else:
                new_value.append(v)
        
        ordered_tuples_int[key] = new_value
        
    ordered_tuples = ordered_tuples_int.copy()
    
    return ordered_tuples


def group_tuples_in_a_sentence(tuples_dict_in): # ont_prop_plural_dict
    # rules have been taken from 'Agregation in natural language generation, by H Dalianis'
    # -subject grouping: all tuples with the same subject are grouped (and)
    # -object grouping: objects that share subject and property are grouped
    grouped_tuples = dict()
    aux_dict = dict()
    sentence = ''
    tuples_dict = tuples_dict_in.copy()
    
    # object grouping 
    for key, tuples in tuples_dict.items():
        # object grouping
        grouped_tuples_by_object = group_tuples_of_same_object_type(tuples) 
        # subject grouping and sentence generation
        sentence = sentence + tuples_list_to_text_with_aggregation(grouped_tuples_by_object) + '. \n\n'
        
    return sentence





# util functions
def kb_solution_to_tuple_specificity_one(assertion_type, pair_of_instances, solution):
    # note that the query solution should contain the fields 'R', 'T1' and 'T2', which will be transformed into tuples
    tuple_ = list()
    tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(pair_of_instances[0])] + ":'" + extract_individual_from_kb_answer(pair_of_instances[0]) + "'")
    tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['R'])] + ":'" + extract_individual_from_kb_answer(solution['R']) + "'")
    tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(pair_of_instances[1])] + ":'" + extract_individual_from_kb_answer(pair_of_instances[1]) + "'")    
    tuple_.append('start:' + str(solution['T1']))
    tuple_.append('end:' + str(solution['T2']))
    tuple_.append(assertion_type) # whether the triple was asserted as affirtmative or negative (e.g., 'it is not a collaboration')
    return tuple_

def kb_solution_to_tuple_specificity_greater_than_one(assertion_type, kb_subject, kb_object, solution):
    # note that the query solution should contain the fields 'R', 'T1' and 'T2', which will be transformed into tuples
    tuple_ = list()
    tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(kb_subject)] + ":'" + extract_individual_from_kb_answer(kb_subject) + "'")
    tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['R'])] + ":'" + extract_individual_from_kb_answer(solution['R']) + "'")
    if extract_raw_uri_from_kb_answer(kb_object):
        # data asserted by means of dul:'hasDataValue', does not have any URI (data cannot be subject, thus this is only needed here)
        tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(kb_object)] + ":'" + extract_individual_from_kb_answer(kb_object) + "'")
    else:
        tuple_.append(":'" + extract_individual_from_kb_answer(kb_object) + "'") 
    tuple_.append('start:' + str(solution['T1']))
    tuple_.append('end:' + str(solution['T2']))
    tuple_.append(assertion_type) # whether the triple was asserted as affirtmative or negative (e.g., 'it is not a collaboration')
    return tuple_

def construct_tuple_from_its_elements(assertion_type, tuple_subject, tuple_object, tuple_relation, tuple_start, tuple_end):
    # note that the query solution should contain the fields 'R', 'T1' and 'T2', which will be transformed into tuples
    tuple_ = list()
    tuple_.append(tuple_subject)
    tuple_.append(tuple_relation)
    tuple_.append(tuple_object)    
    tuple_.append(tuple_start)
    tuple_.append(tuple_end)
    tuple_.append(assertion_type) # whether the triple was asserted as affirtmative or negative (e.g., 'it is not a collaboration')
    return tuple_


def get_pairs_of_instances_of_target_classes(client_rosprolog, ontological_classes_pair, t_locality):
    class_instances = dict()
    instances_pairs = dict()
    instances_pairs["pairs"] = list()
    instances_pairs["intervals"] = list()

    for ontological_class in ontological_classes_pair:
        # positive instances of the classes
        if t_locality: 
            query = client_rosprolog.query("kb_call((triple(I, rdf:type, "+ontological_class+") during[T1, T2], "\
                                        "(T1>="+str(t_locality[0])+", T1=<"+str(t_locality[1])+"; T2>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "\
                                        +str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2; T2=:=inf))).")
        else:
            query = client_rosprolog.query("kb_call(triple(I, rdf:type, "+ontological_class+") during[T1, T2]).")

        for solution in query.solutions():
            #class_instances[solution['I'].split('#')[-1]] =  [solution['T1'], solution['T2']] # instance without ontology iri
            class_instances[solution['I']] =  [solution['T1'], solution['T2']]
            ## print('Found solution. I = %s, T1 = %s, T2 = %s' % (solution['I'], solution['T1'], solution['T2']))
        query.finish()

        # negative instances of the classes
        if t_locality:
            q_ = "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="+str(t_locality[0])+", T1=<"+str(t_locality[1])+"; "\
                "T2>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "+str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2; T2=:=inf))), "\
                "kb_call(triple(D, owl:'sourceIndividual', S) during [T1, T2]), kb_call(triple(D, owl:'assertionProperty', rdf:type) during [T1, T2]), "\
                "kb_call(triple(D, owl:'targetIndividual', "+ontological_class+") during [T1, T2])."
        else:
            q_ = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), \
                kb_call(triple(D, owl:'sourceIndividual', S) during [T1, T2]), \
                kb_call(triple(D, owl:'assertionProperty', rdf:type) during [T1, T2]), \
                kb_call(triple(D, owl:'targetIndividual', "+ontological_class+") during [T1, T2])."
            
        query = client_rosprolog.query(q_)

        for solution in query.solutions():
            #class_instances[solution['S'].split('#')[-1]] =  [solution['T1'], solution['T2']] 
            class_instances[solution['S']] =  [solution['T1'], solution['T2']]
        query.finish()
        
    # create the pairs
    for instanceA, time_intervalA in class_instances.items():
        for instanceB, time_intervalB in class_instances.items():
            if instanceA != instanceB:
                new_pair = [instanceA, instanceB]
                new_pair_copy = new_pair[:]
                new_pair_copy.reverse()
            else:
                continue

            if new_pair not in instances_pairs["pairs"] and new_pair_copy not in instances_pairs["pairs"]:
                instances_pairs["pairs"].append(new_pair)
                instances_pairs["intervals"].append([time_intervalA, time_intervalB]) # instance without ontology iri
            else:
                pass

    ## print(instances_pairs)
    return instances_pairs


def get_ontology_property_and_inverse_dict(client_rosprolog):
    ont_property_inverse_dict = dict()

    query = client_rosprolog.query("kb_call(triple(S, owl:'inverseOf', O))")

    for solution in query.solutions():
        subj_ = solution['S'].split('#')[-1]
        obj_ = solution['O'].split('#')[-1]

        ont_property_inverse_dict[subj_] = obj_
        ont_property_inverse_dict[obj_] = subj_

    query.finish()

    return ont_property_inverse_dict


def invert_tuple_(tuple_in, ont_property_inverse_dict):
    tuple_ = tuple_in.copy()
    tr_ = tuple_.copy()

    ont_uri = extract_uri_label_from_tuple_element(tuple_[1])
    ont_prop = extract_individual_from_tuple_element(tuple_[1])
    if ont_prop in ont_property_inverse_dict:
        tr_[1] = ont_uri + ":'" + ont_property_inverse_dict[ont_prop] + "'"

        tr_[0] = tuple_[2]
        tr_[2] = tuple_[0]
        tr_[3] = tuple_[3]
        tr_[4] = tuple_[4]
        tr_[5] = tuple_[5]
    else:
        pass

    return tr_


def extract_individual_from_tuple_element(tuple_element_in):
    tuple_element = str(tuple_element_in)
    information_to_delete = tuple_element.split(':')[0] # the URI label
    interesting_information = tuple_element.replace(information_to_delete+":", '')
    if "'" in interesting_information:
        ## print(interesting_information)
        interesting_information = ast.literal_eval(interesting_information) # removing extra '' of the strings (e.g. "'has qualiy'" -> "has quality")
    else: 
        pass

    return interesting_information


def extract_uri_label_from_tuple_element(tuple_element_in):
    tuple_element = str(tuple_element_in)
    uri = tuple_element.split(':')[0]
    
    return uri

def extract_individual_from_kb_answer(kb_answer_element_in):
    kb_answer_element = str(kb_answer_element_in)
    individual = kb_answer_element.split('#')[-1]
    
    return individual


def extract_raw_uri_from_kb_answer(kb_answer_element_in):
    kb_answer_element = str(kb_answer_element_in)
    individual = kb_answer_element.split('#')[-1]
    uri = kb_answer_element.replace(individual, '')
    
    return uri


def find_indices_of_related_tuples(tuple_in, tuples_in):
    indices = []
    tuple_ = tuple_in.copy()
    tuples = tuples_in.copy()
    
    for i in range (0, len(tuples)):
        if tuple_[0] == tuples[i][0]:
            indices.append(i)
    
    return indices


def extract_related_tuples(tuples_in, indices):
    tuples = tuples_in.copy()

    related_tuples = [tuples[i] for i in indices]
    
    return related_tuples


def group_tuples_of_same_object_type(tuples_in): # ont_prop_plural_dict
    new_tuples = list()
    tuples = tuples_in.copy()
    
    tuples_cp = tuples.copy()
    while tuples_cp:
        ## print(tuples_cp)
        one_tuple_ = tuples_cp.pop(0)
        indices_to_remove = []
        for i in range(0, len(tuples_cp)):
            if (one_tuple_[0] == tuples_cp[i][0] and one_tuple_[1] == tuples_cp[i][1] and \
               one_tuple_[3] == tuples_cp[i][3] and one_tuple_[4] == tuples_cp[i][4] and \
               one_tuple_[5] == tuples_cp[i][5]):
                # 
                if type(one_tuple_[2]) == list:
                    one_tuple_[2].append(tuples_cp[i][2])
                else:
                    new_list = []
                    new_list.append(one_tuple_[2])
                    new_list.append(tuples_cp[i][2])
                    one_tuple_[2] = new_list.copy()


                indices_to_remove.append(i)
            else:
                continue
        
        one_tuple_[1] = extract_individual_from_tuple_element(one_tuple_[1])
        new_tuples.append(one_tuple_)
        tuples_cp = [tuples_cp[j] for j in range(0, len(tuples_cp)) if j not in indices_to_remove] 
    
    return new_tuples


def tuples_list_to_text_with_aggregation(tuples_in):
    text = ''
    tuples = tuples_in.copy()
    
    for tuple_ in tuples:
        tuple_subject = extract_individual_from_tuple_element(tuple_[0])
        tuple_subject = "'"+re.sub(r"(?<=\w)([A-Z])", r" \1", tuple_subject)+"'" # adding space between words
        
        tuple_relationship = extract_individual_from_tuple_element(tuple_[1])
        if tuple_relationship == "type":
            tuple_relationship = "isATypeOf"
        else:
            pass
        tuple_relationship = re.sub(r"(?<=\w)([A-Z])", r" \1", tuple_relationship) # adding space between words
        tuple_relationship = tuple_relationship.lower() # lowercase
        if tuple_[5] == "negative":
            tuple_relationship = "(not) " + tuple_relationship
        else:
            pass

        if type(tuple_[2]) == list:
            tuple_object = ''
            for obj in tuple_[2]:
                if not tuple_object:
                    tuple_object = extract_individual_from_tuple_element(obj)
                else: 
                    tuple_object = tuple_object + ' and ' + extract_individual_from_tuple_element(obj)
        else:
            tuple_object = extract_individual_from_tuple_element(tuple_[2])

        tuple_object = "'"+re.sub(r"(?<=\w)([A-Z])", r" \1", tuple_object)+"'" # adding space between words

        if (extract_individual_from_tuple_element(tuple_[4]) == 'Infinity'):
            tuple_start = ''
            tuple_end = ''
        else:
            tuple_start = extract_individual_from_tuple_element(tuple_[3])
            tuple_end = extract_individual_from_tuple_element(tuple_[4])

        if not text: 
            # empty text
            if (tuple_start and tuple_end):
                text = tuple_subject + ' ' + tuple_relationship + ' ' + tuple_object + ' ' + 'from ' + tuple_start + ' to ' + tuple_end
            else:
                text = tuple_subject + ' ' + tuple_relationship + ' ' + tuple_object 
        else:
            # non-empty text
            if (tuple_start and tuple_end):
                text = text + ' and ' + tuple_relationship + ' ' + tuple_object + ' ' + 'from ' + tuple_start + ' to ' + tuple_end
            else:
                text = text + ' and ' + tuple_relationship + ' ' + tuple_object 

    return text


def count_string_words(string_, separator):
    string_cp = str(string_)

    number_of_words = len(string_cp.split(separator))

    return number_of_words