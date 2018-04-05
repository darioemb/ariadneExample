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
HybridIOAutomaton getSystem(
	double alpha1_val = 0.02, 
	double beta1_val = 0.03, 
	double beta2_val = 0.03, 
	double gamma1_val = 0.02, 
	double gamma2_val = 0.02,
	double T_val = 4.0, 
	double hmin_val = 1.0, 
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
	RealParameter alpha1("alpha1", alpha1_val);
	RealParameter beta1("beta1", beta1_val);
	RealParameter beta2("beta2", beta2_val);
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
	DiscreteLocation S0("S0");
	HybridIOAutomaton crazy_river = CrazyRiver::getSystem(a, z1, z2, z4,alpha1, beta1, beta2, gamma1, gamma2, T, hmin, hmax, delta, H1, H2, S0);

	//-------- Input valve --------
	DiscreteLocation idle("idle");
	HybridIOAutomaton valve_in = Valve::getSystem(a, T, e_open, e_close, idle);

	// Controller
	DiscreteLocation rising("rising");
	HybridIOAutomaton controller_valve = Controller::getSystem(z1, z2, hmin, H2, delta, e_open, e_close, rising);

	HybridIOAutomaton tank_valve = compose("tanks,valve", crazy_river, valve_in, S0, idle);
	HybridIOAutomaton system = compose("crazy_river", tank_valve, controller_valve, DiscreteLocation("S0,idle"), rising);

	return system;
}
}