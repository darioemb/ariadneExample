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
	HybridIOAutomaton 	system("hydropower");
	RealVariable 		tempo("0");

	//Registration of variables
	system.add_input_var(a);
	system.add_input_var(b);
	system.add_output_var(l);
	system.add_output_var(p);

	system.add_output_var(tempo);
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
	RealVariable 		b,
	RealVariable 		time,
	DiscreteLocation 	day,
	DiscreteLocation 	night,
	DiscreteEvent 		consuming,
	DiscreteEvent 		saving,
	RealParameter 		midday_v)
{
	HybridIOAutomaton 						city("city");
	///Guards
	RealExpression 							midday = time - midday_v; //!< w>=treshold
	RealExpression 							midnight = time - (2 * midday_v);
	///Resets
	std::map<RealVariable, RealExpression> 	reset;
	std::map<RealVariable, RealExpression> 	reset_0;
	///City dynamic
	RealParameter 							c("c",1.0);
	RealExpression 							e_t =c*(b*b/2); // this will be traduced in z(t)=e^(c*t)


	//Registration of variables
	city.add_output_var(b);
	city.add_output_var(time);

	city.add_output_event(saving);
	city.add_output_event(consuming);

	//Registration of locations
	city.new_mode(day);
	city.new_mode(night);

	//Registration of dynamics
	city.set_dynamics(day, b, e_t);
	city.set_dynamics(night, b, -e_t);
	city.set_dynamics(day, time, 1);
	city.set_dynamics(night, time, 1);

	///Resets
	reset[b] = b;
	reset[time] = time;
	reset_0[b] = 0.1;
	reset_0[time] = 0.0;


	city.new_forced_transition(saving, night, day, reset_0, midnight);
	city.new_forced_transition(consuming, day, night, reset, midday);

	return city;
}
}