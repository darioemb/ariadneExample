#pragma once
#include <ariadne.h>

namespace LockSystem1
{
HybridIOAutomaton getSystem(
    RealVariable a1,
    RealVariable z1,
    RealParameter alpha1,
    RealParameter beta1,
    RealParameter baseLevel1,
    RealParameter targetLevel1,    
    DiscreteLocation S01)
{
    // 1.Automaton registration
    HybridIOAutomaton lock_system("lock_system1");

    // 2.Registration of input/output variables
    lock_system.add_input_var(a1);
    lock_system.add_output_var(z1);

    // RealVariable time("0");
    // lock_system.add_output_var(time);
    
    
    DiscreteEvent e_a("e_a");
    lock_system.add_internal_event(e_a);
    DiscreteEvent e_b("e_b");
    lock_system.add_internal_event(e_b);
    
  

    // 4.Registration of Locations
    DiscreteLocation S1("S1");

    lock_system.new_mode(S01); // empty both tank
    lock_system.new_mode(S1); // tank1 hmax=~4m

    // 5.Registration of dynamics
    
    
    //empty initial tank
    lock_system.set_dynamics(S01, z1, beta1 * a1);
    // lock_system.set_dynamics(S01, time,1);
    
    //tank1 not empty
    lock_system.set_dynamics(S1, z1, -alpha1 * z1 + beta1 * a1 );
    // lock_system.set_dynamics(S1, time,1);

    // RealParameter epsilon("epsilon",0.001);

    //guards
    RealExpression guard_a = z1 - baseLevel1;      
    
    RealExpression guard_b = -z1;

    std::map<RealVariable, RealExpression> reset;
    reset[z1] = z1;
    // // reset[time]=time;
    

    lock_system.new_forced_transition(e_a, S01, S1,  reset, guard_a);

    lock_system.new_forced_transition(e_b, S1, S01, reset, guard_b);
        
    return lock_system;
}
}
namespace LockSystem2
{
HybridIOAutomaton getSystem(
    RealVariable a2,
    RealVariable z2,
    RealParameter alpha2,
    RealParameter beta2,
    RealParameter baseLevel2,
    RealParameter targetLevel2,    
    DiscreteLocation S02)
{
    // 1.Automaton registration
    HybridIOAutomaton lock_system("lock_system2");

    // 2.Registration of input/output variables
    lock_system.add_input_var(a2);
    lock_system.add_output_var(z2);    
    
    DiscreteEvent e_c("e_c");
    lock_system.add_internal_event(e_c);
    DiscreteEvent e_d("e_d");
    lock_system.add_internal_event(e_d);
    
  

    // 4.Registration of Locations
    DiscreteLocation S2("S2");

    lock_system.new_mode(S02); // empty both tank
    lock_system.new_mode(S2); // tank1 hmax=~4m

    // 5.Registration of dynamics
    
    
    //empty initial tank
    lock_system.set_dynamics(S02, z2, beta2 * a2);
    
    //tank1 not empty
    lock_system.set_dynamics(S2, z2, -alpha2 * z2 + beta2 * a2 );

    //guards
    RealExpression guard_c = z2 - baseLevel2;      
    
    RealExpression guard_d = -z2;

    std::map<RealVariable, RealExpression> reset;
    reset[z2] = z2;    

    lock_system.new_forced_transition(e_c, S02, S2,  reset, guard_c);

    lock_system.new_forced_transition(e_d, S2, S02, reset, guard_d);
        
    return lock_system;
}
}