#pragma once
#include <ariadne.h>

namespace Valve
{
HybridIOAutomaton getSystem(RealVariable a, RealParameter T, DiscreteEvent e_open,DiscreteEvent e_close, DiscreteLocation idle)
{
    // 1.Automaton registration
    HybridIOAutomaton valve("valve");

    // 2.Registration of input/output
    valve.add_output_var(a);

    // 3.Registration of input/output internal events
    DiscreteEvent e_idle("idle");

    valve.add_input_event(e_open);
    valve.add_input_event(e_close);
    valve.add_internal_event(e_idle);

    // 4.Registration of locations
    DiscreteLocation opening("opening");
    DiscreteLocation closing("closing");

    valve.new_mode(opening);
    valve.new_mode(idle);
    valve.new_mode(closing);

    // 5.Registration of dynamics
    valve.set_dynamics(idle, a, 0.0);
    valve.set_dynamics(closing, a, -1.0 / T);
    valve.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    valve.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    valve.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    valve.new_unforced_transition(e_open, idle, opening);
    valve.new_unforced_transition(e_close, idle, closing);

    return valve;
}
}
