# Finite State Machine for a Surveilance Robot
[Code Documentation](https://salvo-dippolito.github.io/ontological_surveyor/)
## Introduction
This code was developed as a solution to the first assignment of the Experimental Robotics Lab course. It was requested to create a finite state machine architecture for a robot tasked with surveling the floor of a building. To reason about its postion in the floor's environment and to evaluate which rooms to surveil, the robot is introduced as an agent in an ontology with which it interacts by using the [armor package](https://github.com/EmaroLab/armor). The finite state machine architecture was instead stuructured by using [the SMACH package](http://wiki.ros.org/smach).

The robot should move around its environment giving priority to the rooms it hasn't visited for more than a certain threshold of time set by the user. It also has a simulated battery charging and discharging cycle that triggers it to stop executing its tasks so that it can move to a specific room E where it can plug in and recharge.

## Software Architecture
This project depends on one main node, two action servers, the armor service and a robot state notifications node. 
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





## Installation and Running Procedure

