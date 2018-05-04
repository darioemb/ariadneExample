/**
 * @author: Dario Freri
 */
#pragma once
#include <ariadne.h>
#include "controllers.hh"
#include "plants.hh"
#include "valves.hh"
namespace Ariadne
{
HybridIOAutomaton getSystem(
	double alpha_val = 0.1, 
	double beta_val = 1.0, 
	double T_val = 4.0, 
	double baseLevel_val = 0.15, 
	double targetLevel_val = 4.0,
	double delta_val = 0.01)
{
	//System variables
	RealVariable a("a");
	RealVariable z("z");
	

	//System parameters
	RealParameter alpha("alpha", alpha_val);
	RealParameter beta("beta", beta_val);
	RealParameter T("T", T_val);
	RealParameter delta("delta", delta_val);
	RealParameter baseLevel("baseLevel", baseLevel_val);
	RealParameter targetLevel("targetLevel", targetLevel_val);
	

	//ext.events

	DiscreteEvent e_a_open("a_open");
	DiscreteEvent e_a_close("a_close");
	


	// Tanks
	DiscreteLocation S0("S0");
	HybridIOAutomaton lock_system = LockSystem::getSystem(a,z,alpha,beta, baseLevel, targetLevel,S0);

	//-------- Input onevalve --------
	DiscreteLocation a_idle("a_idle");
	HybridIOAutomaton valve = valve::getSystem(a, T, e_a_open, e_a_close, a_idle);

	// Controller
	DiscreteLocation on_first("on_first");
	HybridIOAutomaton controller = controller::getSystem( z,a,baseLevel,targetLevel,delta,e_a_open,e_a_close,on_first);
	
	HybridIOAutomaton tmp = compose("cc",lock_system,controller,S0,on_first);
	std::cout<<tmp<<"\n-------------";
	

	//Compose of all components
	
	HybridIOAutomaton lock_system_valve = compose("lock_system,valve", lock_system, valve, S0, a_idle);
		
	HybridIOAutomaton system = compose("lock_system",lock_system_valve , controller, DiscreteLocation("S0,a_idle"), on_first);

	return system;
}
}