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
	double hmin_val = 1.0, 
	double hmax_val = 2.0, 
	double delta_val = 0.1, 
	double H1_val = 1.90, 
	double H2_val = 1.90,
	double hdmax_val = 2.0,
	double hdmin_val = 1.0)
{
	//System variables
	RealVariable a("a");
	RealVariable z1("z1");
	RealVariable z2("z2");
	RealVariable d("d");

	//System parameters
	RealParameter beta1("beta1", beta1_val);
	RealParameter gamma1("gamma1", gamma1_val);
	RealParameter gamma2("gamma2", gamma2_val);
	RealParameter T("T", T_val);
	RealParameter hmin("hmin", hmin_val);
	RealParameter hmax("hmax", hmax_val);
	RealParameter hdmin("hdmin", hdmin_val);
	RealParameter hdmax("hdmax", hdmax_val);
	RealParameter delta("delta", delta_val);
	RealParameter H1("H1", H1_val);
	RealParameter H2("H2", H2_val);

	DiscreteEvent e_open1("open1");
	DiscreteEvent e_close1("close1");
	DiscreteEvent e_open2("open2");
	DiscreteEvent e_close2("close2");
	DiscreteEvent e_opend("opend");
	DiscreteEvent e_closed("closed");

	// Tanks
	DiscreteLocation no_overflow("no_overflow");
	HybridIOAutomaton crazy_river = crazy_river::getSystem(a, z1, z2, d,beta1, gamma1, gamma2, T,delta, H1, H2, no_overflow);

	//-------- Input valve --------
	DiscreteLocation idle("idle");
	HybridIOAutomaton valve_in = valve::getSystem(a, T, e_open1, e_close1, e_open2, e_close2, e_opend, e_closed, idle);

	// ControllerZ1
	DiscreteLocation rising1("rising1");
	HybridIOAutomaton controller_valve1 = controller_z1::getSystem(z1,hmin, H1, delta, e_open1, e_close1, rising1);


	// ControllerZ2
	DiscreteLocation rising2("rising2");
	HybridIOAutomaton controller_valve2 = controller_z2::getSystem(z2, hmin, H2, delta, e_open2, e_close2, rising2);
	
	// ControllerD
	DiscreteLocation risingd("risingd");
	HybridIOAutomaton controller_valved = controller_d::getSystem(d, hdmin, hdmax, delta, e_opend, e_closed, risingd);

	HybridIOAutomaton tank_valve = compose("tanks,valve", crazy_river, valve_in, no_overflow, idle);
	HybridIOAutomaton tank_ct1 = compose("tanks,valve,ct1",tank_valve,controller_valve1,DiscreteLocation("no_overflow,idle"),rising1);
	HybridIOAutomaton tank_ct2 = compose("tank_ct2", tank_ct1, controller_valve2, DiscreteLocation("no_overflow,idle,rising1"), rising2);
	HybridIOAutomaton system = compose("crazy_river", tank_ct2, controller_valved, DiscreteLocation("no_overflow,idle,rising1,rising2"), risingd);

	return system;
}
}