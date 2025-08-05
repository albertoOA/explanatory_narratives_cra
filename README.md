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
