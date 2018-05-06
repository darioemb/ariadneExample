#pragma once
#include <ariadne.h>

namespace controller
{
HybridIOAutomaton getSystem(
	RealVariable z1, 
	RealVariable z2, 
	RealParameter hmin, 
	RealParameter hmax, 
	RealParameter delta, 
	DiscreteEvent e_open, 
	DiscreteEvent e_close, 
	DiscreteLocation rising)
{
  	// 1.Automaton
	HybridIOAutomaton system("controller");

	// 2.Registration of the input/output variables
	system.add_input_var(z1);
	system.add_input_var(z2);

	// 3.Registration of the events
	system.add_output_event(e_open);
	system.add_output_event(e_close);

	// 4.Registration of the locations
	DiscreteLocation falling("falling");

	system.new_mode(rising);
	system.new_mode(falling);

	// 5.Transitions
	// Invariants
	// RealExpression z_geq_hmin = (hmin - delta - z2) ; // x >= hmin - delta
	// RealExpression z_leq_hmax = (z2 - hmax - delta) ; // x <= hmax + delta

	// system.new_invariant(rising, z_leq_hmax);
	// system.new_invariant(falling, z_geq_hmin);

	// Guards
	RealExpression z_geq_hmax = (z2 - hmax -delta ) ; // z1 >= hmax - delta
	RealExpression z_leq_hmin = (hmin - z2 + delta) ; // z1 <= hmin + delta

	system.new_forced_transition(e_close, rising, falling, z_geq_hmax);
	system.new_forced_transition(e_open, falling, rising, z_leq_hmin);

    return system;
}
}
