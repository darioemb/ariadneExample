#pragma once
#include <ariadne.h>

namespace Controller
{
HybridIOAutomaton getSystem(RealVariable z1, RealVariable z2, RealVariable z3, RealParameter hmin, RealParameter hmax, RealParameter delta, DiscreteEvent e_open, DiscreteEvent e_close, DiscreteLocation rising)
{
  	// 1.Automaton
	HybridIOAutomaton controller("controller");

	// 2.Registration of the input/output variables
	controller.add_input_var(z1);
	controller.add_input_var(z2);
	controller.add_input_var(z3);

	// 3.Registration of the events
	controller.add_output_event(e_open);
	controller.add_output_event(e_close);

	// 4.Registration of the locations
	DiscreteLocation falling("falling");

	controller.new_mode(rising);
	controller.new_mode(falling);

	// 5.Transitions
	// Invariants
	RealExpression z_geq_hmin = (hmin - delta - z1) * (hmin - delta - z2) * (hmin - delta - z3); // x >= hmin - delta
	RealExpression z_leq_hmax = (z1 - hmax - delta) * (z2 - hmax - delta) * (z3 - hmax - delta); // x <= hmax + delta

	controller.new_invariant(rising, z_leq_hmax);
	controller.new_invariant(falling, z_geq_hmin);

	// Guards
	RealExpression z_geq_hmax = (z1 - hmax + delta) * (z2 - hmax + delta) * (z3 - hmax + delta); // z1 >= hmax - delta
	RealExpression z_leq_hmin = (hmin + delta - z1) * (hmin + delta - z2) * (hmin + delta - z3); // z1 <= hmin + delta

	controller.new_unforced_transition(e_close, rising, falling, z_geq_hmax);
	controller.new_unforced_transition(e_open, falling, rising, z_leq_hmin);

    return controller;
}
}
