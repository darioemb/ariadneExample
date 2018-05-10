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
	HybridIOAutomaton system("hydropower");

	//Registration of variables
	system.add_input_var(a);
	system.add_input_var(b);
	system.add_output_var(l);
	system.add_output_var(p);

	//time fixed to first position
	RealVariable tempo("0");
	system.add_output_var(tempo);

	//Registration of events
	//NO events

	//Registration of locations
	system.new_mode(flow);

	//Registration of dynamics
	system.set_dynamics(flow, l, -alpha * a * l + epsilon);
	system.set_dynamics(flow, p, alpha * psi * a - beta * b);
	system.set_dynamics(flow, tempo, 1);

    return system;
}
}
namespace city
{
HybridIOAutomaton getSystem(
	RealVariable b,
	RealVariable time,
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
	city.add_output_var(time);

	city.add_output_event(saving);
	city.add_output_event(consuming);

	//Registration of locations
	city.new_mode(day);
	city.new_mode(night);

	//Registration of dynamics
	city.set_dynamics(day, b, 1.0);
	city.set_dynamics(night,b,-1.0);
	city.set_dynamics(day,time,1.0);
	city.set_dynamics(night,time,1.0);

	std::map<RealVariable, RealExpression> reset;
	reset[b] = b;
	reset[time] = time;

	std::map<RealVariable, RealExpression> reset_0;
	reset_0[b] = b;
	reset_0[time] = 0.0;

	RealExpression midday = time - midday_v; //!< w>=treshold
	RealExpression midnight = time - (2.0 * midday_v);

    city.new_forced_transition(saving, night, day, reset_0, midnight);
    city.new_forced_transition(consuming, day, night, reset, midday);


    return city;
}	
}