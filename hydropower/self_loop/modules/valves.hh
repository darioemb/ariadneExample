#pragma once
#include <ariadne.h>

namespace WaterValve
{
HybridIOAutomaton getSystem(
	RealVariable a, 
	RealParameter T, 
	DiscreteEvent e_a_open, 
	DiscreteEvent e_a_close, 
	DiscreteLocation a_idle)
{
    HybridIOAutomaton valve("w_valve");

	valve.add_output_var(a);

	//Registration of input/output internal events
	DiscreteEvent e_a_idle("a_idle");

	valve.add_input_event(e_a_open);
	valve.add_input_event(e_a_close);
	valve.add_internal_event(e_a_idle);

	//Registration of locations
	DiscreteLocation a_opening("a_opening");
	DiscreteLocation a_closing("a_closing");

	valve.new_mode(a_opening);
	valve.new_mode(a_idle);
	valve.new_mode(a_closing);

	//Registration of dynamics
	valve.set_dynamics(a_idle, a, 0.0);
	valve.set_dynamics(a_closing, a, -1.0 / T);
	valve.set_dynamics(a_opening, a, 1.0 / T);

	//Registration of transitions
	// Guards
	RealExpression a_geq_one = a - 1.0; // a >= 1
	RealExpression a_leq_zero = -a;		// a >= 0

	//Resets
	std::map<RealVariable, RealExpression> rst_a_one;
	rst_a_one[a] = 1.0; // a = 1
	std::map<RealVariable, RealExpression> rst_a_zero;
	rst_a_zero[a] = 0.0; // a = 0

	valve.new_forced_transition(e_a_idle, a_opening, a_idle, rst_a_one, a_geq_one);
	valve.new_forced_transition(e_a_idle, a_closing, a_idle, rst_a_zero, a_leq_zero);

	valve.new_unforced_transition(e_a_open, a_idle, a_opening);
	valve.new_unforced_transition(e_a_close, a_idle, a_closing);


    return valve;
}
}