/**
 * @author: Nicola Dessi'
 */
#pragma once
#include <ariadne.h>
#include "controllers.hh"
#include "tanks.hh"
#include "valves.hh"
namespace Ariadne
{
HybridIOAutomaton getSystem(double g = 9.82, double eta = 0.85, double h = 1.0, double rho = 1.0, double alpha_val = 0.1, double beta_val = 0.1, double epsilon_lower = 0.01, double epsilon_upper = 0.1, double gamma_val = 0.01, double T_val = 2.0, double pmin_val = 1.0, double pmax_val = 10.0, double tmin_val = 25.0, double tmax_val = 30.0)
{
	//System variables
	RealVariable a("a");
	RealVariable b("b");
	RealVariable l("l");
	RealVariable p("p");
	RealVariable t("t");

	//System parameters
	RealParameter alpha("alpha", alpha_val);
	RealParameter beta("beta", beta_val);
	RealParameter psi("psi", g * eta * h * rho);
	RealParameter epsilon("epsilon", 1.0); //Interval(epsilon_lower, epsilon_upper));
	RealParameter gamma("gamma", gamma_val);
	RealParameter T("T", T_val);
	RealParameter pmin("pmin", pmin_val);
	RealParameter pmax("pmax", pmax_val);
	RealParameter tmin("tmin", tmin_val);
	RealParameter tmax("tmax", tmax_val);
	RealParameter delta("delta", 0.1);

	//Initial states
	DiscreteLocation flow("flow");
	DiscreteLocation a_idle("a_idle");
	DiscreteLocation on("on");
	DiscreteLocation p_falling("p_falling");
	DiscreteLocation t_falling("t_falling");

	//External events
	DiscreteEvent e_a_open("e_a_open");
	DiscreteEvent e_a_close("e_a_close");
	DiscreteEvent turn_on("turn_on");
	DiscreteEvent turn_off("turn_off");
	DiscreteEvent e_b_open("e_b_open");
	DiscreteEvent e_b_close("e_b_close");

	// Automaton registration
	HybridIOAutomaton dam = Dam::getSystem(a, b, l, p, t, alpha, beta, psi, gamma, epsilon, flow);

	//------------ Water valve ------------
	HybridIOAutomaton w_valve = WaterValve::getSystem(a, T, e_a_open, e_a_close, a_idle);

	//------------ Temperature valve ------------
	HybridIOAutomaton t_valve = TemperatureValve::getSystem(b, turn_on, turn_off, on);
	//-------- Water valve controller --------
	//Automaton
	HybridIOAutomaton w_controller = WaterController::getSystem(p, pmin, pmax, delta, e_b_open, e_b_close, p_falling);

	//-------- Temperature valve controller --------
	//Automaton
	HybridIOAutomaton t_controller = TemperatureController::getSystem(t, tmin, tmax, delta, turn_on, turn_off, t_falling);

	//Composition
	HybridIOAutomaton dam_valve = compose("dam,valve", dam, w_valve, flow, a_idle);
	HybridIOAutomaton dam_2valve = compose("dam,valve,valve", dam_valve, t_valve, DiscreteLocation("flow,a_idle"), on);
	HybridIOAutomaton dam_2valve_controller = compose("dam,valve,valve,controller", dam_2valve, w_controller, DiscreteLocation("flow,a_idle,off"), p_falling);
	HybridIOAutomaton system = compose("dam", dam_2valve_controller, t_controller, DiscreteLocation("flow,a_idle,off,p_falling"), t_falling);

	return system;
}
}