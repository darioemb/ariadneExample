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
	RealVariable a2("a2");
	RealVariable z1("z1");
	RealVariable z2("z2");
	RealVariable z3("z3");

	//System parameters
	RealParameter alpha1("alpha1", 0.02);
	RealParameter alpha2("alpha2", 0.02);
	RealParameter beta1("beta1", 0.1);
	RealParameter beta2("beta2", 0.1);
	RealParameter T("T", 2.0);
	RealParameter hmin("hmin", 1.0);
	RealParameter hmax("hmax", 2.0);
	RealParameter delta("delta", 0.1);
	RealParameter H("H", 1.90);

	// 1.Automaton registration
	HybridIOAutomaton crazy_river("crazy_river");

	// 2.Registration of input/output variables
	crazy_river.add_input_var(a1);
	crazy_river.add_input_var(a2);
	crazy_river.add_output_var(z1);
	crazy_river.add_output_var(z2);
	crazy_river.add_output_var(z3);

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
	crazy_river.set_dynamics(no_overflow, z1, - alpha1 * z1 + beta1 * a1);
	crazy_river.set_dynamics(no_overflow, z2, - alpha2 * z2 + beta2 * a2);
	crazy_river.set_dynamics(no_overflow, z3, alpha1 * z1 + alpha2 * z2 - beta1 * a1 - beta2 * a2);
	crazy_river.set_dynamics(overflow, z1, 0);
	crazy_river.set_dynamics(overflow, z2, - alpha2 * z2 + beta2 * a2  + (beta1 * a1 - alpha1 * z1));
	crazy_river.set_dynamics(overflow, z3, alpha1 * z1 + alpha2 * z2 - beta1 * a1 - beta2 * a2);

	//guards
	RealExpression guard1 = z1 - H;				   //!< z>=H
	RealExpression guard2 = - beta1 * a1 - alpha1 * z1; //!< z<=beta*a

	std::map<RealVariable, RealExpression> reset1;
	reset1[z1] = z1;
	reset1[z2] = z2;
	reset1[z3] = z3;

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

	//-------- Output valve --------
	// 1.Automaton registration
	HybridIOAutomaton o_valve("o_valve");

	// 2.Registration of input/output
	o_valve.add_output_var(a2);

	// 3.Registration of input/output internal events
	DiscreteEvent e_o_open("o_open");
	DiscreteEvent e_o_close("o_close");
	DiscreteEvent e_o_idle("o_idle");

	o_valve.add_input_event(e_o_open);
	o_valve.add_input_event(e_o_close);
	o_valve.add_internal_event(e_o_idle);

	// 4.Registration of locations
	DiscreteLocation o_opening("o_opening");
	DiscreteLocation o_idle("o_idle");
	DiscreteLocation o_closing("o_closing");

	o_valve.new_mode(o_opening);
	o_valve.new_mode(o_idle);
	o_valve.new_mode(o_closing);

	// 5.Registration of dynamics
	o_valve.set_dynamics(o_idle, a2, 0.0);
	o_valve.set_dynamics(o_closing, a2, -1.0 / T);
	o_valve.set_dynamics(o_opening, a2, 1.0 / T);

	// 6.Registration of transitions
	// Guards
	RealExpression a2_geq_one = a2 - 1.0; // a >= 1
	RealExpression a2_leq_zero = -a2;	 // a >= 0

	// Resets
	std::map<RealVariable, RealExpression> rst_a2_one;
	rst_a2_one[a2] = 1.0; // a = 1
	std::map<RealVariable, RealExpression> rst_a2_zero;
	rst_a2_zero[a2] = 0.0; // a = 0

	o_valve.new_forced_transition(e_o_idle, o_opening, o_idle, rst_a2_one, a2_geq_one);
	o_valve.new_forced_transition(e_o_idle, o_closing, o_idle, rst_a2_zero, a2_leq_zero);

	o_valve.new_unforced_transition(e_o_open, o_idle, o_opening);
	o_valve.new_unforced_transition(e_o_close, o_idle, o_closing);

	//-------- input valve controller --------
	// 1.Automaton
	HybridIOAutomaton controller_o_valve("controller_o_valve");

	// 2.Registration of the input/output variables
	controller_o_valve.add_input_var(z2);

	// 3.Registration of the events
	controller_o_valve.add_output_event(e_o_open);
	controller_o_valve.add_output_event(e_o_close);

	// 4.Registration of the locations
	DiscreteLocation o_rising("o_rising");
	DiscreteLocation o_falling("o_falling");

	controller_o_valve.new_mode(o_rising);
	controller_o_valve.new_mode(o_falling);

	// 5.Transitions
	// Invariants
	RealExpression z2_leq_hmax = z2 - hmax - delta; // x <= hmax + delta
	RealExpression z2_geq_hmin = hmin - delta - z2; // x >= hmin - delta

	controller_o_valve.new_invariant(o_rising, z2_leq_hmax);
	controller_o_valve.new_invariant(o_falling, z2_geq_hmin);

	// Guards
	RealExpression z2_geq_hmax = z2 - hmax + delta; // z1 >= hmax - delta
	RealExpression z2_leq_hmin = hmin + delta - z2; // z1 <= hmin + delta

	controller_o_valve.new_unforced_transition(e_o_close, o_rising, o_falling, z2_geq_hmax);
	controller_o_valve.new_unforced_transition(e_o_open, o_falling, o_rising, z2_leq_hmin);

	HybridIOAutomaton tank_valve1 = compose("tank,i_valve", crazy_river, i_valve, no_overflow, i_idle);
	HybridIOAutomaton tank_valve2 = compose("tank,o_valve", tank_valve1, o_valve, DiscreteLocation("no_overflow,i_idle"), o_idle);
	HybridIOAutomaton tank2 = compose("tank2", tank_valve2, controller_i_valve, DiscreteLocation("no_overflow,i_idle,o_idle"), i_rising);
	HybridIOAutomaton system = compose("crazy_river",tank2,controller_o_valve,DiscreteLocation("no_overflow,i_idle,o_idle,i_rising"),o_rising);

	return system;
}
}