# Lock_System

# Work_in_Progress not yet developed in the right manner.

This system represent a Lock system in which a boat try to overcome the different water level of a river.
We oversimplified on purpose the model in order to grasp the general idea and to get the right analysis.
By abstracting some idea we were able to model a system with two tanks and one controller and make the system able to do 
his job.
The system has to put the boat in a position where it can navigate the river despite the gap between the water level of the river.
In doing so the system required a certain amount of energy that we have to record.

The locks in our case are tanks since ariadne does not provide such feature.
The image below took from the websites: http://www.greatlakes-seaway.com/en/seaway/locks/index.html
![Image of tanks automaton](./lock.jpeg)



# States


          TRANSITION TABLE
         * --------------------
                S0	S1  S2  S3		
         * --------------------
         * a |  S1   -  S3   -	
         * b |  S2  S3  -    -	
         * c |  -   S0  -   S2
         * d |  -   -   S0  S1
  



 
