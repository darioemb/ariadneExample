#pragma once
#include <ariadne.h>

namespace LockSystem
{
HybridIOAutomaton getSystem(
    RealVariable a,
    RealVariable z,
    RealParameter alpha,
    RealParameter beta,
    RealParameter riverLevel,
    RealParameter targetLevel,
    DiscreteEvent  z_leq_riverLevel, 
    DiscreteLocation getin)
{
    // 1.Automaton registration
    HybridIOAutomaton lock_system("lock_system");



    // 2.Registration of input/output variables
    lock_system.add_input_var(a);
    lock_system.add_output_var(z);

    DiscreteEvent z_geq_riverLevel("z_geq_riverLevel");

    lock_system.add_internal_event(z_geq_riverLevel);
    lock_system.add_output_event(z_leq_riverLevel);



    // 4.Registration of Locations
    DiscreteLocation passon("passon");


    lock_system.new_mode(passon);
    lock_system.new_mode(getin);
    

    // 5.Registration of dynamics
    
    
   
    lock_system.set_dynamics(getin, z, -beta*a);
    lock_system.set_dynamics(passon, z, -alpha * z + beta * a );
    

   //guards
    RealExpression guard_z_geq_riverLevel = z - riverLevel;      
    
    RealExpression guard_z_leq_riverLevel = -z + riverLevel;

    std::map<RealVariable, RealExpression> reset;
    reset[z] = z;
    
    
    lock_system.new_forced_transition(z_geq_riverLevel, getin, passon,  reset, guard_z_geq_riverLevel);

    lock_system.new_forced_transition(z_leq_riverLevel, passon, getin, reset, guard_z_leq_riverLevel);
       
    return lock_system;
}
}