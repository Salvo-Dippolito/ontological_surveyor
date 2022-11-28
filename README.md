# Finite State Machine for a Surveilance Robot
[Code Documentation](https://salvo-dippolito.github.io/ontological_surveyor/)
## Introduction
This code was developed as a solution to the first assignment of the Experimental Robotics Lab course. It was requested to create a finite state machine architecture for a robot tasked with surveling the floor of a building. To reason about its postion in the floor's environment and to evaluate which rooms to surveil, the robot is introduced as an agent in an ontology with which it interacts by using the [armor package](https://github.com/EmaroLab/armor). The finite state machine architecture was instead stuructured by using [the SMACH package](http://wiki.ros.org/smach).

The robot should move around its environment giving priority to the rooms it hasn't visited for more than a certain threshold of time set by the user. It also has a simulated battery charging and discharging cycle that triggers it to stop executing its tasks so that it can move to a specific room E where it can plug in and recharge.

## Software Architecture
This project depends on one main node, two action servers, the armor service and a robot state notifications node. The details of how these components work together are shown in this project's documentation.
### UML structure
This is the project's UML:

<img
    src="/images/UML.jpg"
    title="Project UML"
    width="75%" height="75%">
    
The state machine node subscribes to the robot state node through the use of an interface class called AgentState. Through an instance of this class the state machine node is also set up as a client to the two action servers /choose_move_action_server and /execute_move_action_server. All nodes except the robot_state node are set up as clients to the armor service, although the state machine node gets set up through the use of a class called HandleOntology.

### State Machine Structure

<img
    src="/images/statemachine.jpg"
    title="Project State Machine"
    width="75%" height="75%">

There is an initial state called Load Map which only gets accessed at the start of the simulation. In that state the floor ontology is created by following user specifications and the agent is placed in its charging room 'E'. The robot then enters its main routine which has been subdivided in three main actions and consequently three different states: choosing where to go (Choose Move), getting there (Execute Move), and finally surveiling the place for a while (Surveil Room). If at any moment during the execution of each of these states the robot state node where to notify that the battery level was too low then the state machine would transition immediately into its charging state (Charging).
As shown in the documentation, the charging state is actually comprised of a two-phased task: moving back to the charging station and then waiting for the battery to charge. In this state the /execute_move_action_server is called with the chosen location preset as room 'E'.

In this next gif it's possible to see what is shown to the user when the Surveil Room state is interrupted by the Charging state: 
![](https://github.com/Salvo-Dippolito/ontological_surveyor/blob/main/images/interrupt_surveil.gif)


Here is instead shown an example of user interaction in the initial state Load Map:
![](https://github.com/Salvo-Dippolito/ontological_surveyor/blob/main/images/load_map.gif)



## Installation and Running Procedure

### Installation 

These are thesteps to follow to get this simulation to run on your machine:
1. Clone this repository inside your workspace (make sure it is sourced in your .bashrc).
2. Install the [ARMOR](https://github.com/EmaroLab/armor/issues/7) and [Smach-ROS](http://wiki.ros.org/smach/Tutorials/Getting%20Started) package.
  To interact with ARMOR a slightly modified version of the [armor_api](https://github.com/EmaroLab/armor_py_api) has been used. To simplify the installation process,   all the relevant scripts, along with the functions created ad hoc for this applications are present in this repository in the [armor_api](https://github.com/Salvo-Dippolito/ontological_surveyor/tree/main/scripts/armor_api) sub-folder.

3. Run `chmod +x <file_name>` for each pyhton file inside the scripts and armor_api folders.
4. Run `catkin_make` from the root of your workspace.
5. In order to correctly view this project's user interface you'll also need to install the `simple_colors` package by copying the following line on your terminal:

```
pip install simple-colors
```
### Running the Code
After completing the installation procedures you can run the simulation by launching:
```
roslaunch ontological_surveyor surveyor.launch
```



[topological_map](https://github.com/buoncubi/topological_map) 
