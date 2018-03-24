/**
 * @author: Nicola Dessi'
 */
#pragma once
#include <ariadne.h>
namespace Ariadne
{
HybridIOAutomaton getSystem(double g = 9.82, double eta = 0.85, double h = 10.0, double rho = 1000.0, double alpha_val = 0.1, double beta_val = 0.1, double epsilon_lower = 0.00001, double epsilon_upper = 0.0001, double gamma_val = 0.01, double T_val = 4.0, double pmin_val = 1, double pmax_val = 10, double tmin_val = 25.0, double tmax_val = 30.0)
{
	//System variables
	RealVariable l("l");
	RealVariable p("p");
	RealVariable t("t");
	RealVariable a("a");
	RealVariable b("b");

	//System parameters
	RealParameter alpha("alpha", alpha_val);
	RealParameter beta("beta", beta_val);
	RealParameter psi("psi", g * eta * h * rho);
	RealParameter epsilon("epsilon", Interval(epsilon_lower, epsilon_upper));
	RealParameter gamma("gamma", gamma_val);
	RealParameter T("T", T_val);
	RealParameter pmin("pmin", pmin_val);
	RealParameter pmax("pmax", pmax_val);
	RealParameter tmin("tmin", tmin_val);
	RealParameter tmax("tmax", tmax_val);

	// Automaton registration
	HybridIOAutomaton dam("dam");

	//Registration of variables
	dam.add_input_var(a);
	dam.add_input_var(b);
	dam.add_output_var(l);
	dam.add_output_var(p);
	dam.add_output_var(t);

	//Registration of events
	//NO events

	//Registration of locations
	DiscreteLocation flow("flow");
	dam.new_mode(flow);

	//Registration of dynamics
	dam.set_dynamics(flow, l, -alpha * a * l + epsilon);
	dam.set_dynamics(flow, p, psi * a  - alpha * b * p);
	dam.set_dynamics(flow, t, alpha * b * p - gamma * t);

	//------------ Water valve ------------
	HybridIOAutomaton w_valve("w_valve");

	//Registration of input/output
	w_valve.add_output_var(a);

	//Registration of input/output internal events
	DiscreteEvent e_a_open("a_open");
	DiscreteEvent e_a_close("a_close");
	DiscreteEvent e_a_idle("a_idle");

	w_valve.add_input_event(e_a_open);
	w_valve.add_input_event(e_a_close);
	w_valve.add_internal_event(e_a_idle);

	//Registration of locations
	DiscreteLocation a_opening("a_opening");
	DiscreteLocation a_idle("a_idle");
	DiscreteLocation a_closing("a_closing");

	w_valve.new_mode(a_opening);
	w_valve.new_mode(a_idle);
	w_valve.new_mode(a_closing);

	//Registration of dynamics
	w_valve.set_dynamics(a_idle, a, 0.0);
	w_valve.set_dynamics(a_closing, a, -1.0 / T);
	w_valve.set_dynamics(a_opening, a, 1.0 / T);

	//Registration of transitions
	// Guards
	RealExpression a_geq_one = a - 1.0; // a >= 1
	RealExpression a_leq_zero = -a;		// a >= 0

	//Resets
	std::map<RealVariable, RealExpression> rst_a_one;
	rst_a_one[a] = 1.0; // a = 1
	std::map<RealVariable, RealExpression> rst_a_zero;
	rst_a_zero[a] = 0.0; // a = 0

	w_valve.new_forced_transition(e_a_idle, a_opening, a_idle, rst_a_one, a_geq_one);
	w_valve.new_forced_transition(e_a_idle, a_closing, a_idle, rst_a_zero, a_leq_zero);

	w_valve.new_unforced_transition(e_a_open, a_idle, a_opening);
	w_valve.new_unforced_transition(e_a_close, a_idle, a_closing);

	//------------ Temperature valve ------------
	HybridIOAutomaton t_valve("t_valve");

	//Registration of input/output
	t_valve.add_output_var(b);

	//Registration of input/output internal events
	DiscreteEvent e_b_open("b_open");
	DiscreteEvent e_b_close("b_close");
	DiscreteEvent e_b_idle("b_idle");

	t_valve.add_input_event(e_b_open);
	t_valve.add_input_event(e_b_close);
	t_valve.add_internal_event(e_b_idle);

	//Registration of locations
	DiscreteLocation b_opening("b_opening");
	DiscreteLocation b_idle("b_idle");
	DiscreteLocation b_closing("b_closing");

	t_valve.new_mode(b_opening);
	t_valve.new_mode(b_idle);
	t_valve.new_mode(b_closing);

	//Registration of dynamics
	t_valve.set_dynamics(b_idle, b, 0.0);
	t_valve.set_dynamics(b_closing, b, -1.0 / T);
	t_valve.set_dynamics(b_opening, b, 1.0 / T);

	//Registration of transitions
	// Guards
	RealExpression b_geq_one = b - 1.0; // a >= 1
	RealExpression b_leq_zero = -b;		// a >= 0

	//Resets
	std::map<RealVariable, RealExpression> rst_b_one;
	rst_b_one[b] = 1.0; // a = 1
	std::map<RealVariable, RealExpression> rst_b_zero;
	rst_b_zero[b] = 0.0; // a = 0

	t_valve.new_forced_transition(e_b_idle, b_opening, b_idle, rst_b_one, b_geq_one);
	t_valve.new_forced_transition(e_b_idle, b_closing, b_idle, rst_b_zero, b_leq_zero);

	t_valve.new_unforced_transition(e_b_open, b_idle, b_opening);
	t_valve.new_unforced_transition(e_b_close, b_idle, b_closing);

	//-------- Water valve controller --------
	//Automaton
	HybridIOAutomaton w_controller("w_controller");

	//Registration of the input/output variables
	w_controller.add_input_var(p);

	//Registration of the events
	w_controller.add_output_event(e_a_open);
	w_controller.add_output_event(e_a_close);

	//Registration of the locations
	DiscreteLocation p_rising("p_rising");
	DiscreteLocation p_falling("p_falling");

	w_controller.new_mode(p_rising);
	w_controller.new_mode(p_falling);

	//Transitions
	//Invariants
	RealExpression p_geq_pmin = (pmin - p);
	RealExpression p_leq_pmax = (p - pmax); // x <= pmax 

	w_controller.new_invariant(p_rising, p_leq_pmax);
	w_controller.new_invariant(p_falling, p_geq_pmin);

	//Guards
	RealExpression p_geq_pmax = (p - pmax);
	RealExpression p_leq_pmin = (pmin - p);

	w_controller.new_unforced_transition(e_a_close, p_rising, p_falling, p_geq_pmax);
	w_controller.new_unforced_transition(e_a_open, p_falling, p_rising, p_leq_pmin);

	//-------- Temperature valve controller --------
	//Automaton
	HybridIOAutomaton t_controller("t_controller");

	//Registration of the input/output variables
	t_controller.add_input_var(t);

	//Registration of the events
	t_controller.add_output_event(e_b_open);
	t_controller.add_output_event(e_b_close);

	//Registration of the locations
	DiscreteLocation t_rising("t_rising");
	DiscreteLocation t_falling("t_falling");

	t_controller.new_mode(t_rising);
	t_controller.new_mode(t_falling);

	//Transitions
	//Invariants
	RealExpression t_geq_tmin = (tmin - t);
	RealExpression t_leq_tmax = (t - tmax); // x <= hmax + delta

	t_controller.new_invariant(t_rising, t_leq_tmax);
	t_controller.new_invariant(t_falling, t_geq_tmin);

	//Guards
	RealExpression t_geq_tmax = (t - tmax);
	RealExpression t_leq_tmin = (tmin - t);

	t_controller.new_unforced_transition(e_b_close, t_rising, t_falling, t_geq_tmax);
	t_controller.new_unforced_transition(e_b_open, t_falling, t_rising, t_leq_tmin);

	//Composition
	HybridIOAutomaton dam_valve = compose("dam,valve",dam,w_valve,flow,a_idle);
	HybridIOAutomaton dam_2valve = compose("dam,valve,valve",dam_valve,t_valve,DiscreteLocation("flow,a_idle"),b_idle);
	HybridIOAutomaton dam_2valve_controller = compose("dam,valve,valve,controller",dam_2valve,w_controller,DiscreteLocation("flow,a_idle,b_idle"),p_falling);
	HybridIOAutomaton system = compose("dam",dam_2valve_controller,t_controller,DiscreteLocation("flow,a_idle,b_idle,p_falling"),t_falling);

	return system;
}
}