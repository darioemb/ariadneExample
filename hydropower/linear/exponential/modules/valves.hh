#pragma once
#include <ariadne.h>

namespace water_valve
{
HybridIOAutomaton getSystem(
	RealVariable 		a, 
	RealParameter 		T, 
	DiscreteEvent 		e_a_open, 
	DiscreteEvent 		e_a_close, 
	DiscreteLocation 	a_idle)
{
    HybridIOAutomaton 						system("w_valve");
	///Events
	DiscreteEvent 							e_a_idle("a_idle");
	///Locations
	DiscreteLocation 						a_opening("a_opening");
	DiscreteLocation 						a_closing("a_closing");
	//Resets
	std::map<RealVariable, RealExpression> 	rst_a_one;
	std::map<RealVariable, RealExpression> 	rst_a_zero;
	///guards
	RealExpression 							a_geq_one = a - 1.0; // a >= 1
	RealExpression 							a_leq_zero = -a;		// a >= 0


	///Registration of vars
	system.add_output_var(a);

	///Registration of input/output internal events

	system.add_input_event(e_a_open);
	system.add_input_event(e_a_close);
	system.add_internal_event(e_a_idle);

	///Registration of locations

	system.new_mode(a_opening);
	system.new_mode(a_idle);
	system.new_mode(a_closing);

	///Registration of dynamics
	system.set_dynamics(a_idle, a, 0.0);
	system.set_dynamics(a_closing, a, -1.0 / T);
	system.set_dynamics(a_opening, a, 1.0 / T);

	///Resets
	rst_a_one[a] = 1.0; // a = 1
	rst_a_zero[a] = 0.0; // a = 0

	system.new_forced_transition(e_a_idle, a_opening, a_idle, rst_a_one, a_geq_one);
	system.new_forced_transition(e_a_idle, a_closing, a_idle, rst_a_zero, a_leq_zero);

	system.new_unforced_transition(e_a_open, a_idle, a_opening);
	system.new_unforced_transition(e_a_close, a_idle, a_closing);


    return system;
}
}