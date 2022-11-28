# Finite State Machine for a Surveilance Robot
[Code Documentation](https://salvo-dippolito.github.io/ontological_surveyor/)
## Introduction
This code was developed as a solution to the first assignment of the Experimental Robotics Lab course. It was requested to create a finite state machine architecture for a robot tasked with surveling the floor of a building. To reason about its postion in the floor's environment and to evaluate which rooms to surveil the robot is introduced as an agent in an ontology with which it interacts by using the [armor package](https://github.com/EmaroLab/armor). The finite state machine architecture was instead stuructured by using [the SMACH package](http://wiki.ros.org/smach).

The robot should move around its environment giving priority to the rooms it hasn't visited for more than a certain threshold of time set by the user. It also has a simulated battery charging and discharging cycle that triggers it to stop executing its tasks so that it can move to a specific room E where it can plug in and recharge.

## 
