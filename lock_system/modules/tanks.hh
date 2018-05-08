#pragma once
#include <ariadne.h>

namespace LockSystem
{
HybridIOAutomaton getSystem(
    RealVariable a1,
    RealVariable a2,
    RealVariable z1,
    RealVariable z2,
    RealParameter alpha1,
    RealParameter alpha2,
    RealParameter beta1,
    RealParameter beta2,    
    DiscreteLocation S0)
{
    // 1.Automaton registration
    HybridIOAutomaton lock_system("lock_system");

    //tmp
    // RealVariable time("0");

    // 2.Registration of input/output variables
    lock_system.add_input_var(a1);
    lock_system.add_input_var(a2);
    lock_system.add_output_var(z1);
    lock_system.add_output_var(z2);
    
    // lock_system.add_output_var(time);
//VERIFICARE QUESTO STEP
    // 3.Registration of Events
    
    DiscreteEvent e_a("e_a");
    lock_system.add_output_event(e_a);
    DiscreteEvent e_b("e_b");
    lock_system.add_output_event(e_b);
    DiscreteEvent e_c("e_c");
    lock_system.add_output_event(e_c);
    DiscreteEvent e_d("e_d");
    lock_system.add_output_event(e_d);
    
  

    // 4.Registration of Locations
    DiscreteLocation S1("S1");
    DiscreteLocation S2("S2");
    DiscreteLocation S3("S3");
    

    lock_system.new_mode(S0); // empty both tank
    lock_system.new_mode(S1); // tank1 hmax=~4m
    lock_system.new_mode(S2); // tank2 hmax=~7m
    lock_system.new_mode(S3); // both not empty

    // 5.Registration of dynamics
    
    
    //lock_system.set_dynamics(S0, time, 1);
    //empty initial tank
    lock_system.set_dynamics(S0, z1, beta1 * a1);
    lock_system.set_dynamics(S0, z2, beta2 * a2);
    
    //lock_system.set_dynamics(S1, time, 1);
    //tank1 not empty
    lock_system.set_dynamics(S1, z1, -alpha1 * z1 + beta1 * a1 );
    lock_system.set_dynamics(S1, z2, beta2 * a2);
    
    // lock_system.set_dynamics(S2, time, 1);
    
    //empty 2 not empty  tank1 empty
    lock_system.set_dynamics(S2, z1, beta1 * a1);
    lock_system.set_dynamics(S2, z2, -alpha2 * z2 + beta2 * a2);
    // lock_system.set_dynamics(S3, time, 1);
    //both tank with water
    lock_system.set_dynamics(S3, z1, -alpha1 * z1 + beta1 * a1 );
    lock_system.set_dynamics(S3, z2, -alpha2 * z2 + beta2 * a2);
    

    //guards
    
    RealExpression guard_a = z1 - 0.0001;      
    
    RealExpression guard_b = z2 - 0.0001;

    RealExpression guard_c = -z1 + 0.0001;

    RealExpression guard_d = -z2 + 0.0001;        


    

    std::map<RealVariable, RealExpression> reset;
    reset[z1] = z1;
    reset[z2] = z2;
    
    // // reset[time] = time;

    /**
         * 			TRANSITION TABLE
         * --------------------
         * 	 |	S0	S1	S2  S3		
         * --------------------
         * a |	S1	-	S3	-	
         * b |	S2	S3	-	-	
         * c |  -   S0  -   S2
         * d |  -   -   S0  S1
         */
    lock_system.new_forced_transition(e_a, S0, S1,  reset, guard_a);
    lock_system.new_forced_transition(e_b, S0, S2, reset, guard_b);

    lock_system.new_forced_transition(e_c, S1, S0, reset, guard_c);
    lock_system.new_forced_transition(e_b, S1, S3, reset, guard_b);

    lock_system.new_forced_transition(e_a, S2, S3, reset, guard_a);
    lock_system.new_forced_transition(e_d, S2, S0, reset, guard_d);

    lock_system.new_forced_transition(e_c, S3, S2, reset, guard_c);
    lock_system.new_forced_transition(e_d, S3, S1, reset, guard_d);



    
    return lock_system;
}
}