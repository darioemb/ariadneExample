#pragma once
#include <ariadne.h>

namespace oneValve
{
HybridIOAutomaton getSystem(
    RealVariable a1, 
    RealParameter T, 
    DiscreteEvent e_a1_open,
    DiscreteEvent e_a1_close, 
    DiscreteLocation a1_idle)
{
    // 1.Automaton registration
    HybridIOAutomaton valve("valve1");

    // 2.Registration of input/output
    valve.add_output_var(a1);

    // 3.Registration of input/output internal events
    
     //DiscreteEvent e_a1_open("a1_open");
     //DiscreteEvent e_a1_close("a1_close");
     DiscreteEvent e_a1_idle("a1_idle");

    valve.add_input_event(e_a1_open);
    valve.add_input_event(e_a1_close);
    valve.add_internal_event(e_a1_idle);

    // 4.Registration of locations
    // DiscreteLocation a1_idle("a1_idle");
     DiscreteLocation a1_opening("a1_opening");
     DiscreteLocation a1_closing("a1_closing");

    valve.new_mode(a1_opening);
    valve.new_mode(a1_idle);
    valve.new_mode(a1_closing);

    // 5.Registration of dynamics
    valve.set_dynamics(a1_idle, a1, 0.0);
    valve.set_dynamics(a1_closing, a1, -1.0 / T);
    valve.set_dynamics(a1_opening, a1, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a1_geq_one = a1 - 1.0; // a >= 1
    RealExpression a1_leq_zero = -a1;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a1_one;
    rst_a1_one[a1] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a1_zero;
    rst_a1_zero[a1] = 0.0; // a = 0

    valve.new_forced_transition(e_a1_idle, a1_opening, a1_idle, rst_a1_one, a1_geq_one);
    valve.new_forced_transition(e_a1_idle, a1_closing, a1_idle, rst_a1_zero, a1_leq_zero);

    valve.new_unforced_transition(e_a1_open, a1_idle, a1_opening);
    valve.new_unforced_transition(e_a1_close, a1_idle, a1_closing);

    return valve;
}
}
namespace twoValve
{
HybridIOAutomaton getSystem(
    RealVariable a2, 
    RealParameter T, 
    DiscreteEvent e_a2_open,
    DiscreteEvent e_a2_close, 
    DiscreteLocation a2_idle)
{
    // 1.Automaton registration
    HybridIOAutomaton valve("valve2");

    // 2.Registration of input/output
    valve.add_output_var(a2);

    // 3.Registration of input/output internal events
    //DiscreteEvent e_a2_open("a2_open");
    //DiscreteEvent e_a2_close("a2_close");
    DiscreteEvent e_a2_idle("a2_idle");

    valve.add_input_event(e_a2_open);
    valve.add_input_event(e_a2_close);
    valve.add_internal_event(e_a2_idle);

    // 4.Registration of locations
    DiscreteLocation a2_opening("a2_opening");
    DiscreteLocation a2_closing("a2_closing");
    //DiscreteLocation a2_idle("a2_idle");

    valve.new_mode(a2_opening);
    valve.new_mode(a2_idle);
    valve.new_mode(a2_closing);

    // 5.Registration of dynamics
    valve.set_dynamics(a2_idle, a2, 0.0);
    valve.set_dynamics(a2_closing, a2, -1.0 / T);
    valve.set_dynamics(a2_opening, a2, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a2_geq_one = a2 - 1.0; // a >= 1
    RealExpression a2_leq_zero = -a2;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a2_one;
    rst_a2_one[a2] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a2_zero;
    rst_a2_zero[a2] = 0.0; // a = 0

    valve.new_forced_transition(e_a2_idle, a2_opening, a2_idle, rst_a2_one, a2_geq_one);
    valve.new_forced_transition(e_a2_idle, a2_closing, a2_idle, rst_a2_zero, a2_leq_zero);

    valve.new_unforced_transition(e_a2_open, a2_idle, a2_opening);
    valve.new_unforced_transition(e_a2_close, a2_idle, a2_closing);

    return valve;
}
}