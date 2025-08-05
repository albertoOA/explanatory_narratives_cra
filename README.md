# XONCRA
Explanatory Ontology-based Narratives for Collaborative Robotics and Adapatation.

### Basic information
This ROS package has been tested with Ubuntu 18.04 and ROS Melodic 1.14.12, although it should work with other distributions. Note that the most important aspect is to make sure that you are using Python3, which might require to setup a virtual environment. 


### Python3 virtual environment configuration
In the package, we already provide a virtual environment but it was built for our computer and it will probably not work in yours. You can easily delete it and create and configure your own environment executing the following commands in a terminal.

```
cd <explanatory_narratives_cra_folder>/python_environment
python3 -m venv xoncra
source xoncra/bin/activate

pip3 install catkin_pkg rospkg pyyaml future textstat

```

### Generating explanatory narratives: an example task *'filling a tray'*
In order to generate the explanatory narratives, it is essential a running knowledge base containing the knowledge to be narrated. Hence, make sure that you have launched [know_cra](https://github.com/albertoOA/know_cra/) with the validation neem or your own episodic memories. Then, you can run the test.

```
rosrun explanatory_narratives_cra axon_test.py
```

If you have an empty knowledge base running, it is also possible to assert the *validation neem* knowledge before generating the narratives. 

```
rosrun explanatory_narratives_cra assert_general_neems_cra.py
```


### Generating contrastive explanatory narratives: an example task *'bringing an object'*

This example deals with a use case in which a robot receives a command: 'bring a drink', which is ambiguous, since there are several drinks in the scene. Hence, the robot would generate using symbolic planning as many plans as drinks there are in the environment, and then it would compare the plans and generate an explanation of which plan is better according to its internal ontological criteria. 

In order to generate the contrastive explanatory narratives, it is essential a running knowledge base, which might contain part of the knowledge to be narrated. Hence, make sure that you have launched [know_cra](https://github.com/albertoOA/know_cra/) with the validation neem for *'bringing an object'* or your own episodic memories. In this case, you might want to launch: 

```
roslaunch know_cra map_cra_cs_bringing_object_neem_manual_assertion.launch
```

The previous command would open a simple knowledge base and it will assert some facts about the objects in the environment (e.g. their qualities, such as which drinks are healthier, colder, farther away, etc.). In order to have different plans (i.e. to bring each drink), it is necessary to open a planning problem in which the goal is to bring a drink, generate the plan, and then change the goal and generate a new plan for each drink. The following launch file first generates the plans to bring all the different drinks in the map (there are two, tea and cola), asserts the knowedge about those plans (e.g. their qualities), and generates the final contrastive narrative comparing in pairs the different drinks and then the different plans:

```
roslaunch explanatory_narratives_cra c_narratives_bringing_object_plan_disambiguation.launch
```



### Evaluating contrastive explanatory narratives against plain explanatory narratives
In order to evaluate how plain narratives (axon-generated) differ from contrastive narratives (generated with acxon), an experiment was conducted using different planning domains and multiple problems [1,2]. The idea here is to run a generic knowledge base in which knowledge about two different plans will be stored for later comparison, contrastive selection and narration. Note that this process shall be repeated for as many pairs as specified in the experiment [2], but here we will only show how to do it for one pair of plans. 

The next example assumes that ***know-plan*** is already working on your system, following the instructions on the gihtub README: [https://github.com/albertoOA/know_plan](https://github.com/albertoOA/know_plan).


Let's start running a generic knowledge base: 

```
roslaunch know_plan map_cra_cs_generic.launch
```

Now it is time to generate a couple of plans for plan comparison and contrastive narration. Given a domain (rovers) and two problems (instances 1 and 6), we will run a planner to solve both problems, obtaining two different plans with their own properties that will be stored in a knowledge base. Then, some ontological rules defined within *know-plan* are applied, performing the comparison of the two plans and asserting the inferred knowledge into the knowledge base (e.g. which plan is better). In the following launch file, it is necessary to configure the variables 'domain_name' (rovers, in this case) and 'domain_problem'. Note that the value of 'problem' will first be 'instance-1', we will launch it, and then we will launch it again with the second problem (instance-6). will do this process for one of the problems, then we will need to adjust 

```
roslaunch know_plan rosplan_cra_cs_generic.launch 
``` 

Finally, the narratives for the pair of plans are generated using *axon* and *acxon*, and some objective metrics are computed for each of them. It will be necessary to configure the value of the variable 'domain_name' (rovers) and 'problem_name' (instances_1-6) in the launch file before running it: 

```
roslaunch explanatory_narratives_cra test_axon_vs_acxon_objective_metrics.launch
```

The execution will generate some txt files containing the different narratives and also csv files with the metrics results. Remember that in order to conduct the complete experiment, one might repeat the previous process changing the domain and/or the problems. 



**[1]** Alberto Olivares-Alarcos, Sergi Foix, Júlia Borràs, Gerard Canal, and Guillem Alenyà. 2024. *Ontological Modeling and Reasoning for Comparison and Contrastive Narration of Robot Plans.* In Proceedings of the 23rd International Conference on Autonomous Agents and Multiagent Systems (AAMAS '24). International Foundation for Autonomous Agents and Multiagent Systems, 2405–2407.

**[2]** Alberto Olivares-Alarcos, Sergi Foix, Júlia Borràs, Gerard Canal, and Guillem Alenyà. 2025. *Ontological Foundations for Contrastive Explanatory Narration of Robot Plans.* Information Sciences, under review.