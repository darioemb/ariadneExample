#pragma once
#include <ariadne.h>

namespace LockSystem
{
HybridIOAutomaton getSystem(
    RealVariable a,
    RealVariable z,
    RealParameter alpha,
    RealParameter beta,
    RealParameter baseLevel,
    RealParameter targetLevel,    
    DiscreteLocation S0)
{
    // 1.Automaton registration
    HybridIOAutomaton lock_system("lock_system");

    // 2.Registration of input/output variables
    lock_system.add_input_var(a);
    lock_system.add_output_var(z);

    // RealVariable time("0");
    // lock_system.add_output_var(time);
    
    
    DiscreteEvent e_a("e_a");
    lock_system.add_internal_event(e_a);
    DiscreteEvent e_b("e_b");
    lock_system.add_internal_event(e_b);
    
  

    // 4.Registration of Locations
    DiscreteLocation S1("S1");

    lock_system.new_mode(S0); // empty both tank
    lock_system.new_mode(S1); // tank1 hmax=~4m

    // 5.Registration of dynamics
    
    
    //empty initial tank
    lock_system.set_dynamics(S0, z, beta * a);
    // lock_system.set_dynamics(S0, time,1);
    
    //tank1 not empty
    lock_system.set_dynamics(S1, z, -alpha * z + beta * a );
    // lock_system.set_dynamics(S1, time,1);

    // RealParameter epsilon("epsilon",0.001);

    //guards
    RealExpression guard_a = z - baseLevel;      
    
    RealExpression guard_b = -z + baseLevel;

    std::map<RealVariable, RealExpression> reset;
    reset[z] = z;
    // // reset[time]=time;
    

    lock_system.new_forced_transition(e_a, S0, S1,  reset, guard_a);

    lock_system.new_forced_transition(e_b, S1, S0, reset, guard_b);
        
    return lock_system;
}
}