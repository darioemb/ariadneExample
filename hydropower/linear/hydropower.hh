/**
 * @author: Nicola Dessi'
 */
#pragma once
#include <ariadne.h>
#include "controllers.hh"
#include "plants.hh"
#include "valves.hh"
namespace Ariadne
{
HybridIOAutomaton getSystem(
	double g = 9.81, 
	double eta = 0.85, 
	double h = 10.0, 
	double rho = 1000.0, 
	double alpha_val = 0.01, 
	double beta_val = 0.1, 
	double epsilon_val = 0.2, 
	double gamma_val = 0.01, 
	double T_val = 2.0, 
	double pmin_val = 200.0, 
	double pmax_val = 4000.0)
{
	//System variables
	RealVariable a("a");
	RealVariable b("b");
	RealVariable l("l");
	RealVariable p("p");

	//System parameters
	RealParameter alpha("alpha", alpha_val);
	RealParameter beta("beta", beta_val);
	RealParameter psi("psi", g * eta * h * rho);
	RealParameter epsilon("epsilon", epsilon_val); //Interval(epsilon_lower, epsilon_upper));
	RealParameter gamma("gamma", gamma_val);
	RealParameter T("T", T_val);
	RealParameter pmin("pmin", pmin_val);
	RealParameter pmax("pmax", pmax_val);
	RealParameter delta("delta", 0.1);
	RealParameter midday("midday", 12);

	//Initial states
	DiscreteLocation flow("flow");
	DiscreteLocation a_idle("a_idle");
	DiscreteLocation on("on");
	DiscreteLocation p_falling("p_falling");
	DiscreteLocation day("day");
	DiscreteLocation night("night");

	//External events
	DiscreteEvent e_a_open("a_open");
	DiscreteEvent e_a_close("a_close");
	DiscreteEvent turn_on("turn_on");
	DiscreteEvent turn_off("turn_off");
	DiscreteEvent e_b_open("b_open");
	DiscreteEvent e_b_close("b_close");
	DiscreteEvent saving("saving");
	DiscreteEvent consuming("consuming");

	// Automaton registration
	HybridIOAutomaton hydropower = hydropower::getSystem(a, b, l, p, alpha, beta, psi, gamma, epsilon, flow);
	RealVariable time("time");
	HybridIOAutomaton city = city::getSystem(b,time, day, night, consuming, saving, midday);

	//------------ Water valve ------------
	HybridIOAutomaton w_valve = WaterValve::getSystem(a, T, e_a_open, e_a_close, a_idle);

	//-------- Water valve controller --------
	//Automaton
	HybridIOAutomaton w_controller = WaterController::getSystem(p, pmin, pmax, delta, e_a_open, e_a_close, p_falling);

	//Composition
	HybridIOAutomaton hydropower_valve = compose("hydropower,valve", hydropower, w_valve, flow, a_idle);
	HybridIOAutomaton hydropower_valve_controller = compose("hydropower,valve,controller", hydropower_valve, w_controller, DiscreteLocation("flow,a_idle"), p_falling);
	HybridIOAutomaton system = compose("hydropower", hydropower_valve_controller, city, DiscreteLocation("flow,a_idle,p_falling"), day);

	return system;
}
}