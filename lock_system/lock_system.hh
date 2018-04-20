/**
 * @author: Dario Freri
 */
#pragma once
#include <ariadne.h>
#include "controllers.hh"
#include "tanks.hh"
#include "valves.hh"
namespace Ariadne
{
HybridIOAutomaton getSystem(
	double alpha1_val = 0.08, 
	double alpha2_val = 0.08,
	double beta1_val = 0.08, 
	double beta2_val = 0.08, 
	double T_val = 4.0, 
	double hmin1_val = 0.001, 
	double hmax1_val = 4.0001,
	double hmin2_val = 4.01, 
	double hmax2_val = 7.0,  
	double delta_val = 0.01,
	double baseLevel1_val = 0.0,
	double baseLevel2_val = 3.91)
{
	//System variables
	RealVariable a1("a1");
	RealVariable a2("a2");
	RealVariable z1("z1");
	RealVariable z2("z2");
	

	//System parameters
	RealParameter alpha1("alpha1", alpha1_val);
	RealParameter alpha2("alpha2", alpha2_val);
	RealParameter beta1("beta1", beta1_val);
	RealParameter beta2("beta2", beta2_val);
	RealParameter T("T", T_val);
	RealParameter hmin1("hmin1", hmin1_val);
	RealParameter hmax1("hmax1", hmax1_val);
	RealParameter hmin2("hmin2", hmin2_val);
	RealParameter hmax2("hmax2", hmax2_val);
	RealParameter delta("delta", delta_val);
	RealParameter baseLevel1("baseLevel1", baseLevel1_val);
	RealParameter baseLevel2("baseLevel2", baseLevel2_val);
	

	//ext.events

	DiscreteEvent e_a1_open("a1_open");
	DiscreteEvent e_a1_close("a1_close");
	DiscreteEvent e_a2_open("a2_open");
	DiscreteEvent e_a2_close("a2_close");
	


	// Tanks
	DiscreteLocation S0("S0");
	HybridIOAutomaton lock_system = LockSystem::getSystem(a1,a2, z1, z2, alpha1, alpha2, beta1, beta2, baseLevel1, baseLevel2,S0);

	//-------- Input onevalve --------
	DiscreteLocation a1_idle("a1_idle");
	HybridIOAutomaton valve1 = oneValve::getSystem(a1, T, e_a1_open, e_a1_close, a1_idle);

	//-------- Input Twovalve --------
	DiscreteLocation a2_idle("a2_idle");
	HybridIOAutomaton valve2 = twoValve::getSystem(a2, T, e_a2_open, e_a2_close, a2_idle);

	// Controller
	DiscreteLocation on_first("on_first");
	HybridIOAutomaton controller = controller::getSystem( z1,z2,a1, a2, hmin1,hmax1,hmin2,hmax2,delta,e_a1_open,e_a1_close, e_a2_open, e_a2_close,on_first);
	
	

	//Compose of all components
	
	HybridIOAutomaton lock_system_valve1 = compose("lock_system,valve", lock_system, valve1, S0, a1_idle);
	
	HybridIOAutomaton lock_system_valve2 = compose("lock_system,valve,valve",lock_system_valve1, valve2, DiscreteLocation("S0,a1_idle"), a2_idle);
	
	HybridIOAutomaton system = compose("lock_system",lock_system_valve2 , controller, DiscreteLocation("S0,a1_idle,a2_idle"), on_first);

	return system;
}
}