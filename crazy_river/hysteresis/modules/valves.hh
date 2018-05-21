#pragma once
#include <ariadne.h>

namespace valve
{
HybridIOAutomaton getSystem(
    RealVariable a, 
    RealParameter T, 
    DiscreteEvent e_open1,
    DiscreteEvent e_close1,
    DiscreteEvent e_open2,
    DiscreteEvent e_close2,
    DiscreteEvent e_opend,
    DiscreteEvent e_closed, 
    DiscreteLocation idle)
{
    // 1.Automaton registration
    HybridIOAutomaton system("valve");

    // 2.Registration of input/output
    system.add_output_var(a);

    // 3.Registration of input/output internal events
    DiscreteEvent e_idle("e_idle");

    system.add_input_event(e_open1);
    system.add_input_event(e_close1);
    system.add_input_event(e_open2);
    system.add_input_event(e_close2);
    system.add_input_event(e_opend);
    system.add_input_event(e_closed);
    system.add_internal_event(e_idle);

    // 4.Registration of locations
    DiscreteLocation opening("opening");
    DiscreteLocation closing("closing");

    system.new_mode(opening);
    system.new_mode(idle);
    system.new_mode(closing);

    // 5.Registration of dynamics
    system.set_dynamics(idle, a, 0.0);
    system.set_dynamics(closing, a, -1.0 / T);
    system.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    system.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    system.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    system.new_unforced_transition(e_open1, idle, opening);
    system.new_unforced_transition(e_close1, idle, closing);
    system.new_unforced_transition(e_open2, idle, opening);
    system.new_unforced_transition(e_close2, idle, closing);
    system.new_unforced_transition(e_opend, idle, opening);
    system.new_unforced_transition(e_closed, idle, closing);

    return system;
}
}
