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
	double beta1_val = 0.8, 
	double beta2_val = 0.8, 
	double T_val = 4.0,  
	double delta_val = 0.01,
	double baseLevel1_val = 0.15,
	double baseLevel2_val = 2.0,
	double targetLevel2_val = 4.0)
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
	RealParameter delta("delta", delta_val);
	RealParameter baseLevel1("baseLevel1", baseLevel1_val);
	RealParameter baseLevel2("baseLevel2", baseLevel2_val);
	RealParameter targetLevel2("targetLevel2", targetLevel2_val);
	

	//ext.events

	DiscreteEvent e_a1_open("a1_open");
	DiscreteEvent e_a1_close("a1_close");
	DiscreteEvent e_a2_open("a2_open");
	DiscreteEvent e_a2_close("a2_close");
	


	// Tanks
	DiscreteLocation S01("S01");
	DiscreteLocation S02("S02");
	HybridIOAutomaton lock_system1 = LockSystem1::getSystem(a1,z1,alpha1,beta1, baseLevel1, baseLevel2,S01);
	HybridIOAutomaton lock_system2 = LockSystem2::getSystem(a2,z2,alpha2,beta2, baseLevel2, targetLevel2,S02);

	//-------- Input onevalve --------
	DiscreteLocation a1_idle("a1_idle");
	HybridIOAutomaton valve1 = valve1::getSystem(a1, T, e_a1_open, e_a1_close, a1_idle);

	//-------- Input Twovalve --------
	DiscreteLocation a2_idle("a2_idle");
	HybridIOAutomaton valve2 = valve2::getSystem(a2, T, e_a2_open, e_a2_close, a2_idle);

	// Controller
	DiscreteLocation on_first("on_first");
	HybridIOAutomaton controller = controller::getSystem( z1,z2,a1, a2, baseLevel1,baseLevel2,baseLevel2,targetLevel2,delta,e_a1_open,e_a1_close, e_a2_open, e_a2_close,on_first);
	
	

	//Compose of all components

	HybridIOAutomaton lock_system = compose("lock_system", lock_system1, lock_system2, S01,S02);

	std::cout<<lock_system<<"\n";
	
	HybridIOAutomaton lock_system_valve1 = compose("lock_system,valve", lock_system, valve1, DiscreteLocation("S01,S02"), a1_idle);
	
	HybridIOAutomaton lock_system_valve2 = compose("lock_system,valve,valve",lock_system_valve1, valve2, DiscreteLocation("S01,S02,a1_idle"), a2_idle);
	
	HybridIOAutomaton system = compose("lock_system",lock_system_valve2 , controller, DiscreteLocation("S01,S02,a1_idle,a2_idle"), on_first);

	return system;
}
}