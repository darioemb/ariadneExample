#pragma once
#include <ariadne.h>

namespace Dam
{
//!< alpha1_i_val : uscita z_i; beta_i_val : ingresso z_i
HybridIOAutomaton getSystem(RealVariable a, RealVariable b, RealVariable l, RealVariable p, RealVariable t, RealParameter alpha, RealParameter beta, RealParameter psi, RealParameter gamma, RealParameter epsilon, DiscreteLocation flow)
{
   // Automaton registration
	HybridIOAutomaton dam("dam");

	//Registration of variables
	dam.add_input_var(a);
	dam.add_input_var(b);
	dam.add_output_var(l);
	dam.add_output_var(p);
	dam.add_output_var(t);

	//time fixed to first position
	//RealVariable time("0");
	// dam.add_output_var(time);

	//Registration of events
	//NO events

	//Registration of locations
	dam.new_mode(flow);

	//Registration of dynamics
	dam.set_dynamics(flow, l, -alpha * a * l + epsilon);
	dam.set_dynamics(flow, p, alpha * psi * a - beta * b * p);
	dam.set_dynamics(flow, t, beta * b * p - gamma * t);
	// dam.set_dynamics(flow, time, 1);

    return dam;
}
}
