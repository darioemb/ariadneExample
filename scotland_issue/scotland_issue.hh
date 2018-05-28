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
	double alpha_val = 0.4,
	double beta_val = 0.8,
	double gamma_val = 0.4,
	double delta_val = 0.1,
	double tau_val = 1.2,
	double H_val = 4.0,
	double Kp_val = 10.0,
	double stormySize_val = 1.0,
	double treshold_val = 20.0,
	double T_val = 4.0)
{
	//System variables
	RealVariable a("a");
	RealVariable z("z");
	RealVariable d("d");
	RealVariable w("w");

	//System parameters
	RealParameter alpha("alpha", alpha_val);
	RealParameter beta("beta", beta_val);
	RealParameter gamma("gamma", gamma_val);
	RealParameter delta("delta", delta_val);
	RealParameter T("T", T_val);
	RealParameter H("H", H_val);
	RealParameter H_c("H_c",2.0);
	RealParameter treshold("treshold", treshold_val);
	RealParameter Kp("Kp",Kp_val);
	RealParameter tau("tau",tau_val);

	DiscreteEvent e_open("open");
	DiscreteEvent e_close("close");
	DiscreteEvent sunny("sunny");//TODO: delete this

	DiscreteLocation no_overflow("no_overflow");
	HybridIOAutomaton tanks = scotland_issue::getSystem(a,d,w,z,alpha,beta,gamma,H,no_overflow);

	DiscreteLocation tick("tick");
	HybridIOAutomaton weather = stormy_weather::getSystem(w,sunny,tick,treshold);

	DiscreteLocation stabilizing("stabilizing");
	HybridIOAutomaton controller = controller_proportional::getSystem(a,z,delta,Kp,H_c,tau,stabilizing);

	HybridIOAutomaton scotland = compose("scotland",tanks,weather,no_overflow,tick);
	HybridIOAutomaton system = compose("scotland_issue",scotland,controller,DiscreteLocation("no_overflow,tick"),stabilizing);
	
	return system;
}
}