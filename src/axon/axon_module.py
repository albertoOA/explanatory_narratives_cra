#!/usr/bin/env python3

"""
What is this code?
  - axon module for ontology-based explanations. 
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
from prolog.prolog_module import *
from rosprolog_client import PrologException, Prolog


def retrieve_narrative_tuples_(client_rosprolog, ontological_classes, t_locality, specificity):
    tuples = dict()

    ont_property_inverse_dict = get_ontology_property_and_inverse_dict(client_rosprolog)

    for ontological_class in ontological_classes:
        # extract the instances of the target classes
        class_instances_uri_dict, class_instances_interval_dict = get_instances_of_target_class(client_rosprolog, ontological_class, t_locality) 

        #for assertion_type, instances_list in class_instances_interval_dict.items():
        for class_instance, instance_time_interval in class_instances_interval_dict.items():
            # initialize the dictionary key
            tuples[class_instance] = list()

            if (specificity < 1 or specificity > 3):
                print("Error while executing retrieve_narrative_tuples, inccorrect specificity value.")
            else:
                if (specificity >= 1):
                    retrieve_narrative_tuples_specificity_one(client_rosprolog, class_instance, instance_time_interval, tuples, ont_property_inverse_dict, class_instances_uri_dict[class_instance])
                else:
                    pass
                if (specificity >= 2):
                    retrieve_narrative_tuples_specificity_two(client_rosprolog, class_instance, instance_time_interval, tuples, ont_property_inverse_dict, class_instances_uri_dict[class_instance])
                else:
                    pass
                if (specificity == 3):
                    retrieve_narrative_tuples_specificity_three(client_rosprolog, class_instance, instance_time_interval, tuples, ont_property_inverse_dict, class_instances_uri_dict[class_instance])
                else:
                    pass
            
    return tuples


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
def retrieve_narrative_tuples_specificity_one(client_rosprolog, class_instance, instance_time_interval, tuples, ont_property_inverse_dict, semantic_map_uri):
    # note that the variable tuples is modified within this function
    for r in range(2):
        ##print(r)
        if (r == 0):
            q_ = "kb_call(triple("+semantic_map_uri+":'"+class_instance+"', R, E) during [T1, T2]), "\
                 "not(dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R))."
            assertion_type = "affirmative"
        else: 
            q_ = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'sourceIndividual', "+semantic_map_uri+":'"+class_instance+"') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2]), "\
                 "not(dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R))."
            assertion_type = "negative"

        query = client_rosprolog.query(q_)

        for solution in query.solutions():
            class_ont_uri = semantic_map_uri
            tr_ = kb_solution_to_tuple_(assertion_type, class_ont_uri, class_instance, solution) 
            #print(tr_)
            if (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[0]) and extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[1]) and tuples[class_instance]):
                tr_[3] = ''
                tr_[4] = ''
            else:
                pass

            tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
            if tr_ in tuples[class_instance]:
                #print("Tuple already in list.")
                pass
            elif tr_inv_ in tuples[class_instance]:
                #print("Inverse tuple already in list.")
                pass
            else:
                tuples[class_instance].append(tr_)

        query.finish()


def retrieve_narrative_tuples_specificity_two(client_rosprolog, class_instance, instance_time_interval, tuples, ont_property_inverse_dict, semantic_map_uri):
    for r in range(2):
        ##print(r)
        if (r == 0):
            q_ = "kb_call(triple("+semantic_map_uri+":'"+class_instance+"', R, E) during [T1, T2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R)."
            assertion_type = "affirmative"
        else: 
            q_ = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'sourceIndividual', "+semantic_map_uri+":'"+class_instance+"') during [T1, T2]), "\
                 "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', R)."
            assertion_type = "negative"

        query = client_rosprolog.query(q_)

        for solution in query.solutions():
            class_ont_uri = semantic_map_uri
            tr_ = kb_solution_to_tuple_(assertion_type, class_ont_uri, class_instance, solution)
            #print(tr_)
            if (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[0]) and extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[1])):
                tr_[3] = ''
                tr_[4] = ''
            else:
                pass

            tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
            if tr_ in tuples[class_instance]:
                #print("Tuple already in list.")
                pass
            elif tr_inv_ in tuples[class_instance]:
                #print("Inverse tuple already in list.")
                pass
            else:
                tuples[class_instance].append(tr_)

        query.finish()


def retrieve_narrative_tuples_specificity_three(client_rosprolog, class_instance, instance_time_interval, tuples, ont_property_inverse_dict, semantic_map_uri):
    for r in range(4):
        ##print(r)
        if (r == 0):
            q_ = "kb_call(triple("+semantic_map_uri+":'"+class_instance+"', Rx, Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), "\
                 "kb_call((triple(Ex, R, E) during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2; T2=:=inf)))." # ; TODO: add 'T2=:=inf' (after '=<T2' and before '))).')
            assertion_type = "affirmative"
        elif (r == 1):
            q_ = "kb_call(triple("+semantic_map_uri+":'"+class_instance+"', Rx, Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), " \
                 "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2; T2=:=inf))), "\
                 "kb_call(triple(D, owl:'sourceIndividual', Ex) during [T1, T2]), " \
                 "kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2])." # ; TODO: add 'T2=:=inf' (after '=<T2' and before '))),')
            assertion_type = "negative"
        elif (r == 2):
            q_ = "kb_call(triple(Dx, Rdx, owl:'NegativePropertyAssertion') during [Tx1, Tx2]), "\
                 "kb_call(triple(Dx, owl:'sourceIndividual', "+semantic_map_uri+":'"+class_instance+"') during [Tx1, Tx2]), " \
                 "kb_call(triple(Dx, owl:'assertionProperty', Rx) during [Tx1, Tx2]), kb_call(triple(Dx, owl:'targetIndividual', Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), "\
                 "kb_call((triple(Ex, R, E) during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2; T2=:=inf)))." # ; TODO: add 'T2=:=inf' (after '=<T2' and before '))).')
            assertion_type = "affirmative"
        else: 
            q_ = "kb_call(triple(Dx, Rdx, owl:'NegativePropertyAssertion') during [Tx1, Tx2]), "\
                 "kb_call(triple(Dx, owl:'sourceIndividual', "+semantic_map_uri+":'"+class_instance+"') during [Tx1, Tx2]), " \
                 "kb_call(triple(Dx, owl:'assertionProperty', Rx) during [Tx1, Tx2]), kb_call(triple(Dx, owl:'targetIndividual', Ex) during [Tx1, Tx2]), "\
                 "dif('http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Rx), "\
                 "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="+str(instance_time_interval[0])+", T1=<"+str(instance_time_interval[1])+"; "\
                 "T2>="+str(instance_time_interval[0])+", T2=<"+str(instance_time_interval[1])+"; "+str(instance_time_interval[0])+">=T1, "+str(instance_time_interval[1])+"=<T2; T2=:=inf))), "\
                 "kb_call(triple(D, owl:'sourceIndividual', Ex) during [T1, T2]), kb_call(triple(D, owl:'assertionProperty', R) during [T1, T2]), "\
                 "kb_call(triple(D, owl:'targetIndividual', E) during [T1, T2])." # ; TODO: add 'T2=:=inf' (after '=<T2' and before '))),')
            assertion_type = "negative"

        query = client_rosprolog.query(q_)


        for solution in query.solutions():
            class_ = extract_individual_from_kb_answer(solution["Ex"]) #solution["Ex"].split('#')[-1]
            class_ont_uri = owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution["Ex"])]
            tr_ = kb_solution_to_tuple_(assertion_type, class_ont_uri, class_, solution)
            ##print(tr_)
            if (extract_individual_from_tuple_element(tr_[3]) == str(instance_time_interval[0]) and extract_individual_from_tuple_element(tr_[4]) == str(instance_time_interval[1])):
                tr_[3] = ''
                tr_[4] = ''
            else:
                pass

            tr_inv_ = invert_tuple_(tr_, ont_property_inverse_dict)
            #for existent_tr in tuples[class_instance]:
            if tr_ in tuples[class_instance]:
            #if all(item in tr_ for item in existent_tr):
            #if tr_[0] == existent_tr[0] and tr_[1] == existent_tr[1] and tr_[2] == existent_tr[2]:
                #print("Tuple already in list.")
                pass
            elif tr_inv_ in tuples[class_instance]:
            #elif all(item in tr_inv_ for item in existent_tr):
            #if tr_inv_[0] == existent_tr[0] and tr_inv_[1] == existent_tr[1] and tr_inv_[2] == existent_tr[2]:
                #print("Inverse tuple already in list.")
                pass
            else:
                tuples[class_instance].append(tr_)

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
            ##print("if instance is object")
            if invert_tuple_(tuple_, ont_prop_dict) in casted_tuples: # avoid repetitions when there are more than 2 target instances
                ##print("if inverted in casted")
                pass
            else:
                casted_tuples.append(invert_tuple_(tuple_, ont_prop_dict))
        else:
            ##print("if instance is not object")
            tuple_property = extract_individual_from_tuple_element(tuple_[1])
            concatenated_casted_tuples = []
            casted_tuples_cp = casted_tuples.copy()
            [concatenated_casted_tuples.extend(el) for el in casted_tuples_cp]

            if tuple_property in ont_prop_dict and any(ont_prop_dict[tuple_property] in s for s in concatenated_casted_tuples):
                ##print("if prop in casted")
                if invert_tuple_(tuple_, ont_prop_dict) in casted_tuples: # avoid repetitions e.g. isWorsePlanThan
                    ##print("if inverted in casted")
                    pass
                else:
                    ##print("if inverted not in casted")
                    casted_tuples.append(invert_tuple_(tuple_, ont_prop_dict))
            else:
                ##print("if prop not in casted")
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
def kb_solution_to_tuple_(assertion_type, class_ont_uri, class_instance, solution):
    # note that the query solution should contain the fields 'R', 'E', 'T1' and 'T2', which will be transformed into tuples
    tuple_ = list()

    tuple_.append(class_ont_uri+":"+class_instance)
    tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['R'])] + ':' + extract_individual_from_kb_answer(solution['R']))
    if extract_raw_uri_from_kb_answer(solution['E']):
        # data asserted by means of dul:'hasDataValue', does not have any URI (data cannot be subject, thus this is only needed here)
        tuple_.append(owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['E'])] + ":" + extract_individual_from_kb_answer(solution['E']))
    else:
        tuple_.append(":" + extract_individual_from_kb_answer(solution['E'])) 
    tuple_.append('start:' + str(solution['T1']))
    tuple_.append('end:' + str(solution['T2']))
    tuple_.append(assertion_type) # whether the triple was asserted as affirtmative or negative (e.g., 'it is not a collaboration')
    return tuple_


def get_instances_of_target_class(client_rosprolog, ontological_class, t_locality):
    class_instances = dict()
    class_instances_uri = dict()

    # positive instances of the classes
    if t_locality: 
        query = client_rosprolog.query("kb_call((triple(I, rdf:type, "+ontological_class+") during[T1, T2], "\
                                    "(T1>="+str(t_locality[0])+", T1=<"+str(t_locality[1])+"; T2>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "\
                                    +str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2; T2=:=inf))).") # ; TODO: add 'T2=:=inf' (after '=<T2' and before '))).')
    else:
        query = client_rosprolog.query("kb_call(triple(I, rdf:type, "+ontological_class+") during[T1, T2]).")

    for solution in query.solutions():
        class_instances[solution['I'].split('#')[-1]] =  [solution['T1'], solution['T2']] # instance without ontology uri
        class_instances_uri[solution['I'].split('#')[-1]] = owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['I'])]
        #print('Found solution. I = %s, T1 = %s, T2 = %s' % (solution['I'], solution['T1'], solution['T2']))
    query.finish()

    # negative instances of the classes
    if t_locality:
        q_ = "kb_call((triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2], (T1>="+str(t_locality[0])+", T1=<"+str(t_locality[1])+"; "\
            "T2>="+str(t_locality[0])+", T2=<"+str(t_locality[1])+"; "+str(t_locality[0])+">=T1, "+str(t_locality[1])+"=<T2; T2=:=inf))), "\
            "kb_call(triple(D, owl:'sourceIndividual', S) during [T1, T2]), kb_call(triple(D, owl:'assertionProperty', rdf:type) during [T1, T2]), "\
            "kb_call(triple(D, owl:'targetIndividual', "+ontological_class+") during [T1, T2])." # ; TODO: add 'T2=:=inf' (after '=<T2' and before '))),')
    else:
        q_ = "kb_call(triple(D, Rd, owl:'NegativePropertyAssertion') during [T1, T2]), \
            kb_call(triple(D, owl:'sourceIndividual', S) during [T1, T2]), \
            kb_call(triple(D, owl:'assertionProperty', rdf:type) during [T1, T2]), \
            kb_call(triple(D, owl:'targetIndividual', "+ontological_class+") during [T1, T2])."

    query = client_rosprolog.query(q_)
    for solution in query.solutions():
        class_instances[solution['S'].split('#')[-1]] =  [solution['T1'], solution['T2']] 
        class_instances_uri[solution['S'].split('#')[-1]] = owl_uri_to_label_dict[extract_raw_uri_from_kb_answer(solution['S'])]
    query.finish()
    
    return class_instances_uri, class_instances


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
        tr_[1] = ont_uri+':'+ont_property_inverse_dict[ont_prop]

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
        ##print(tuples_cp)
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
        ## tuple_subject = "'"+re.sub(r"(?<=\w)([A-Z])", r" \1", tuple_subject)+"'" # adding space between words
        
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

        ## tuple_object = "'"+re.sub(r"(?<=\w)([A-Z])", r" \1", tuple_object)+"'" # adding space between words

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