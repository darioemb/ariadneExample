#pragma once
#include <ariadne.h>

namespace hydropower
{
//!< alpha1_i_val : uscita z_i; beta_i_val : ingresso z_i
HybridIOAutomaton getSystem(
	RealVariable a, 
	RealVariable b, 
	RealVariable l, 
	RealVariable p, 
	RealParameter alpha, 
	RealParameter beta, 
	RealParameter psi, 
	RealParameter gamma, 
	RealParameter epsilon, 
	DiscreteLocation flow)
{
   // Automaton registration
	HybridIOAutomaton hydropower("hydropower");

	//Registration of variables
	hydropower.add_input_var(a);
	hydropower.add_input_var(b);
	hydropower.add_output_var(l);
	hydropower.add_output_var(p);

	//time fixed to first position
	RealVariable time("0");
	hydropower.add_output_var(time);

	//Registration of events
	//NO events

	//Registration of locations
	hydropower.new_mode(flow);

	//Registration of dynamics
	hydropower.set_dynamics(flow, l, -alpha * a * l + epsilon);
	hydropower.set_dynamics(flow, p, alpha * psi * a - beta * b * p);
	hydropower.set_dynamics(flow, time, 1);

    return hydropower;
}
}
namespace city
{
HybridIOAutomaton getSystem(
	RealVariable b,
	DiscreteLocation day,
	DiscreteLocation night,
	DiscreteEvent consuming,
	DiscreteEvent saving,
	RealParameter midday_v)
{
   // Automaton registration
	HybridIOAutomaton city("city");

	//Registration of variables
	city.add_output_var(b);

	city.add_output_event(saving);
	city.add_output_event(consuming);

	//Registration of locations
	city.new_mode(day);
	city.new_mode(night);

	//Registration of dynamics
	city.set_dynamics(day, b, 1);
	city.set_dynamics(night,b,-1);

	std::map<RealVariable, RealExpression> reset;
    reset[b] = b;

    RealExpression midday = b - midday_v; //!< w>=treshold
    RealExpression midnight = -b;

    city.new_forced_transition(saving, night, day, reset, midnight);
    city.new_forced_transition(consuming, day, night, reset, midday);


    return city;
}	
}