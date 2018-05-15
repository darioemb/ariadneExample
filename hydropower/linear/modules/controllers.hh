#pragma once
#include <ariadne.h>

namespace power_controller
{
HybridIOAutomaton getSystem(
	RealVariable 		p, 
	RealParameter 		pmin, 
	RealParameter 		pmax, 
	RealParameter 		delta, 
	DiscreteEvent 		e_a_open, 
	DiscreteEvent 		e_a_close, 
	DiscreteLocation 	p_falling)
{
  	///Automaton
	HybridIOAutomaton 	system("p_controller");
	///Guards
	RealExpression 		p_geq_pmax = (p - pmax-delta);
	RealExpression 		p_leq_pmin = (pmin - p+delta);
	///Locations
	DiscreteLocation 	p_rising("p_rising");

	// 2.Registration of the input/output variables
	system.add_input_var(p);

	// 3.Registration of the events
	system.add_output_event(e_a_open);
	system.add_output_event(e_a_close);

	// 4.Registration of the locations

	system.new_mode(p_rising);
	system.new_mode(p_falling);

	//Guards

	system.new_forced_transition(e_a_close, p_rising, p_falling, p_geq_pmax);
	system.new_forced_transition(e_a_open, p_falling, p_rising, p_leq_pmin);

    return system;
}
}