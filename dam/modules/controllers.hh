#pragma once
#include <ariadne.h>

namespace WaterController
{
HybridIOAutomaton getSystem(RealVariable p, RealParameter pmin, RealParameter pmax, RealParameter delta, DiscreteEvent e_open, DiscreteEvent e_close, DiscreteLocation p_falling)
{
  	// 1.Automaton
	HybridIOAutomaton controller("w_controller");

	// 2.Registration of the input/output variables
	controller.add_input_var(p);

	// 3.Registration of the events
	controller.add_output_event(e_open);
	controller.add_output_event(e_close);

	// 4.Registration of the locations
	DiscreteLocation p_rising("p_rising");

	controller.new_mode(p_rising);
	controller.new_mode(p_falling);

	// 5.Transitions
	// Invariants
	RealExpression p_geq_pmin = (pmin - p);
	RealExpression p_leq_pmax = (p - pmax); // x <= pmax

	controller.new_invariant(p_rising, p_leq_pmax);
	controller.new_invariant(p_falling, p_geq_pmin);

	//Guards
	RealExpression p_geq_pmax = (p - pmax);
	RealExpression p_leq_pmin = (pmin - p);

	controller.new_unforced_transition(e_close, p_rising, p_falling, p_geq_pmax);
	controller.new_unforced_transition(e_open, p_falling, p_rising, p_leq_pmin);

    return controller;
}
}
namespace TemperatureController
{
HybridIOAutomaton getSystem(RealVariable t, RealParameter tmin, RealParameter tmax, RealParameter delta, DiscreteEvent turn_on, DiscreteEvent turn_off, DiscreteLocation t_falling)
{
  	//Automaton
	HybridIOAutomaton controller("t_controller");

	//Registration of the input/output variables
	controller.add_input_var(t);

	//Registration of the events
	controller.add_output_event(turn_on);
	controller.add_output_event(turn_off);

	//Registration of the locations
	DiscreteLocation t_rising("t_rising");

	controller.new_mode(t_rising);
	controller.new_mode(t_falling);

	//Transitions
	//Invariants
	RealExpression t_geq_tmin = (tmin - t);
	RealExpression t_leq_tmax = (t - tmax); // x <= hmax + delta

	controller.new_invariant(t_rising, t_leq_tmax);
	controller.new_invariant(t_falling, t_geq_tmin);

	//Guards
	RealExpression t_geq_tmax = (t - tmax);
	RealExpression t_leq_tmin = (tmin - t);

	controller.new_unforced_transition(turn_off, t_rising, t_falling, t_geq_tmax);
	controller.new_unforced_transition(turn_on, t_falling, t_rising, t_leq_tmin);

    return controller;
}
}