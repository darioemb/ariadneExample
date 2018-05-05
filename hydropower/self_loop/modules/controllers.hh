#pragma once
#include <ariadne.h>

namespace WaterController
{
HybridIOAutomaton getSystem(
	RealVariable p, 
	RealParameter pmin, 
	RealParameter pmax, 
	RealParameter delta, 
	DiscreteEvent e_a_open, 
	DiscreteEvent e_a_close, 
	DiscreteLocation p_falling)
{
  	// 1.Automaton
	HybridIOAutomaton controller("w_controller");

	// 2.Registration of the input/output variables
	controller.add_input_var(p);

	// 3.Registration of the events
	controller.add_output_event(e_a_open);
	controller.add_output_event(e_a_close);

	// 4.Registration of the locations
	DiscreteLocation p_rising("p_rising");

	controller.new_mode(p_rising);
	controller.new_mode(p_falling);

	// 5.Transitions
	// Invariants
	// RealExpression p_geq_pmin = (pmin - p);
	// RealExpression p_leq_pmax = (p - pmax); // x <= pmax

	// controller.new_invariant(p_rising, p_leq_pmax);
	// controller.new_invariant(p_falling, p_geq_pmin);

	//Guards
	RealExpression p_geq_pmax = (p - pmax);
	RealExpression p_leq_pmin = (pmin - p);

	controller.new_forced_transition(e_a_close, p_rising, p_falling, p_geq_pmax);
	controller.new_forced_transition(e_a_open, p_falling, p_rising, p_leq_pmin);

    return controller;
}
}