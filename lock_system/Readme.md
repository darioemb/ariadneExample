# Lock_System

# Work_in_Progress NoT Yet developed in the right manner.

This system represent a Lock system in which a boat try to overcome the different water level of a river.
We oversimplified on purpose the model in order to grasp the general idea and to get the right analysis.
By abstracting some idea we were able to model a system with two tanks and one controller and make the system able to do 
his job.
The system has to put the boat in a position where it can navigate the river despite the gap between the water level of the river.
In doing so the system required a certain amount of energy that we have to record.

The locks in our case are tanks since ariadne does not provide such feature.

![Image of tanks automaton](.images/lock.jpeg)

# HOW TO
compiling:


1. cd build
2. cmake ..
3. make

Then you can run the tutorial with:

./scotland VERBOSITY

where the (optional) VERBOSITY value shall be positive. 
