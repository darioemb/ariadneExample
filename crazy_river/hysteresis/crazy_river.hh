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
	double beta1_val = 0.4, 
	double gamma1_val = 0.04, 
	double gamma2_val = 0.04,
	double T_val = 4.0, 
	double hmin_val = 0.5, 
	double hmax_val = 2.0, 
	double delta_val = 0.1, 
	double H1_val = 1.90, 
	double H2_val = 1.90)
{
	//System variables
	RealVariable a("a");
	RealVariable z1("z1");
	RealVariable z2("z2");
	RealVariable z4("z4");

	//System parameters
	RealParameter beta1("beta1", beta1_val);
	RealParameter gamma1("gamma1", gamma1_val);
	RealParameter gamma2("gamma2", gamma2_val);
	RealParameter T("T", T_val);
	RealParameter hmin("hmin", hmin_val);
	RealParameter hmax("hmax", hmax_val);
	RealParameter delta("delta", delta_val);
	RealParameter H1("H1", H1_val);
	RealParameter H2("H2", H2_val);

	DiscreteEvent e_open("open");
	DiscreteEvent e_close("close");

	// Tanks
	DiscreteLocation no_overflow("no_overflow");
	HybridIOAutomaton crazy_river = CrazyRiver::getSystem(a, z1, z2, z4,beta1, gamma1, gamma2, T,delta, H1, H2, no_overflow);

	//-------- Input valve --------
	DiscreteLocation idle("idle");
	HybridIOAutomaton valve_in = Valve::getSystem(a, T, e_open, e_close, idle);

	// Controller
	DiscreteLocation rising("rising");
	HybridIOAutomaton controller_valve = Controller::getSystem(z1, z2, hmin, H2, delta, e_open, e_close, rising);

	HybridIOAutomaton tank_valve = compose("tanks,valve", crazy_river, valve_in, no_overflow, idle);
	HybridIOAutomaton system = compose("crazy_river", tank_valve, controller_valve, DiscreteLocation("no_overflow,idle"), rising);

	return system;
}
}