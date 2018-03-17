/**
 * @author: Nicola Dessi'
 */
#pragma once
#include <ariadne.h>

namespace Ariadne
{
HybridIOAutomaton getSystem()
{
	//System variables
	RealVariable a1("a1");
	RealVariable z1("z1");
	//RealVariable a2("a2");
	RealVariable z2("z2");

	//System parameters
	RealParameter alpha1("alpha1", 0.02);
	RealParameter alpha2("alpha2", 0.02);
	RealParameter beta1("beta1", Interval(0.3, 0.32863));
	RealParameter beta2("beta2", Interval(0.3, 0.32863));
	RealParameter T("T", 4.0);
	RealParameter hmin("hmin", 5.75);
	RealParameter hmax("hmax", 7.75);
	RealParameter delta("delta", 0.1);
	RealParameter H("H", 7.74);

	// 1.Automaton registration
	HybridIOAutomaton crazy_river("crazy_river");

	// 2.Registration of input/output variables
	crazy_river.add_input_var(a1);
	//crazy_river.add_input_var(a2);
	crazy_river.add_output_var(z1);
	crazy_river.add_output_var(z2);

	// 3.Registration of Events
	DiscreteEvent e_overflow("e_overflow");
	crazy_river.add_output_event(e_overflow);
	DiscreteEvent e_no_overflow("e_no_overflow");
	crazy_river.add_output_event(e_no_overflow);

	// 4.Registration of Locations
	DiscreteLocation no_overflow("no_overflow");
	DiscreteLocation overflow("overflow");

	crazy_river.new_mode(no_overflow);
	crazy_river.new_mode(overflow);

	// 5.Registration of dynamics
	crazy_river.set_dynamics(no_overflow, z1, -alpha1 * z1 + beta1 * a1);
	crazy_river.set_dynamics(no_overflow, z2, -alpha2 * z2 /*+ beta2 * a2*/);
	crazy_river.set_dynamics(overflow, z1, - alpha1 * z1);
	crazy_river.set_dynamics(overflow, z2, - alpha2 * z2 /*+ beta2 * a2*/  + alpha1 * z1);

	//guards
	RealExpression guard1 = z1 - H;				   //!< z>=H
	RealExpression guard2 = - beta1 * a1 - alpha1 * z1; //!< z<=beta*a

	std::map<RealVariable, RealExpression> reset1;
	reset1[z1] = z1;
	reset1[z2] = z2;

	crazy_river.new_forced_transition(e_overflow, no_overflow, overflow, reset1, guard1);
	crazy_river.new_forced_transition(e_no_overflow, overflow, no_overflow, reset1, guard2);

	//-------- Input valve --------
	// 1.Automaton registration
	HybridIOAutomaton i_valve("i_valve");

	// 2.Registration of input/output
	i_valve.add_output_var(a1);

	// 3.Registration of input/output internal events
	DiscreteEvent e_i_open("i_open");
	DiscreteEvent e_i_close("i_close");
	DiscreteEvent e_i_idle("i_idle");

	i_valve.add_input_event(e_i_open);
	i_valve.add_input_event(e_i_close);
	i_valve.add_internal_event(e_i_idle);

	// 4.Registration of locations
	DiscreteLocation i_opening("i_opening");
	DiscreteLocation i_idle("i_idle");
	DiscreteLocation i_closing("i_closing");

	i_valve.new_mode(i_opening);
	i_valve.new_mode(i_idle);
	i_valve.new_mode(i_closing);

	// 5.Registration of dynamics
	i_valve.set_dynamics(i_idle, a1, 0.0);
	i_valve.set_dynamics(i_closing, a1, -1.0 / T);
	i_valve.set_dynamics(i_opening, a1, 1.0 / T);

	// 6.Registration of transitions
	// Guards
	RealExpression a_geq_one = a1 - 1.0; // a >= 1
	RealExpression a_leq_zero = -a1;	 // a >= 0

	// Resets
	std::map<RealVariable, RealExpression> rst_a_one;
	rst_a_one[a1] = 1.0; // a = 1
	std::map<RealVariable, RealExpression> rst_a_zero;
	rst_a_zero[a1] = 0.0; // a = 0

	i_valve.new_forced_transition(e_i_idle, i_opening, i_idle, rst_a_one, a_geq_one);
	i_valve.new_forced_transition(e_i_idle, i_closing, i_idle, rst_a_zero, a_leq_zero);

	i_valve.new_unforced_transition(e_i_open, i_idle, i_opening);
	i_valve.new_unforced_transition(e_i_close, i_idle, i_closing);

	//-------- input valve controller --------
	// 1.Automaton
	HybridIOAutomaton controller_i_valve("controller_i_valve");

	// 2.Registration of the input/output variables
	controller_i_valve.add_input_var(z1);

	// 3.Registration of the events
	controller_i_valve.add_output_event(e_i_open);
	controller_i_valve.add_output_event(e_i_close);

	// 4.Registration of the locations
	DiscreteLocation i_rising("i_rising");
	DiscreteLocation i_falling("i_falling");

	controller_i_valve.new_mode(i_rising);
	controller_i_valve.new_mode(i_falling);

	// 5.Transitions
	// Invariants
	RealExpression z_leq_hmax = z1 - hmax - delta; // x <= hmax + delta
	RealExpression z_geq_hmin = hmin - delta - z1; // x >= hmin - delta

	controller_i_valve.new_invariant(i_rising, z_leq_hmax);
	controller_i_valve.new_invariant(i_falling, z_geq_hmin);

	// Guards
	RealExpression z_geq_hmax = z1 - hmax + delta; // z1 >= hmax - delta
	RealExpression z_leq_hmin = hmin + delta - z1; // z1 <= hmin + delta

	controller_i_valve.new_unforced_transition(e_i_close, i_rising, i_falling, z_geq_hmax);
	controller_i_valve.new_unforced_transition(e_i_open, i_falling, i_rising, z_leq_hmin);

	HybridIOAutomaton tank_valve = compose("tank,i_valve", crazy_river, i_valve, no_overflow, i_idle);
	HybridIOAutomaton system = compose("crazy_river", tank_valve, controller_i_valve, DiscreteLocation("no_overflow,i_idle"), i_rising);
	return system;
}
}