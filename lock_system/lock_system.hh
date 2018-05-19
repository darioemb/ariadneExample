/**
 * @author: Dario Freri, Nicola Dessi'
 */
#pragma once
#include <ariadne.h>
#include "controllers.hh"
#include "plants.hh"

namespace Ariadne
{
HybridIOAutomaton getSystem(
	double alpha_val = 0.1, 
	double beta_val = 0.8, 
	double T_val = 4.0, 
	double riverLevel_val = 1.0, 
	double kp_val = 1.0,
	double tau_val = 1.25,
	double thr_val =7.0,
	double targetLevel_val = 4.0,
	double delta_val = 0.00)
{
	//System variables
	RealVariable a("a");
	RealVariable z("z");
	RealVariable t("t");

	

	//System parameters
	RealParameter alpha("alpha", alpha_val);
	RealParameter beta("beta", beta_val);
	RealParameter T("T", T_val);
	RealParameter delta("delta", delta_val);
	RealParameter riverLevel("riverLevel", riverLevel_val);
	RealParameter targetLevel("targetLevel", targetLevel_val);
	RealParameter kp("kp", kp_val);
	RealParameter tau("tau", tau_val);
	RealParameter thr("thr", thr_val);

	

	//ext.events

	DiscreteEvent  z_leq_riverLevel(" z_leq_riverLevel");
	DiscreteLocation stabilizing("stabilizing");
	DiscreteLocation getin("getin");



	// Tanks
	
	HybridIOAutomaton lock_system = LockSystem::getSystem(a,z,alpha,beta,riverLevel,targetLevel,z_leq_riverLevel,getin);

	
	// Controller
	
	HybridIOAutomaton controller = controller::getSystem(z,a,t,thr,kp,tau,targetLevel,delta,z_leq_riverLevel,stabilizing);
	
	

	//Compose of all components
	
	
	HybridIOAutomaton system = compose("lock_system",lock_system , controller, getin, stabilizing);

	return system;
}
}