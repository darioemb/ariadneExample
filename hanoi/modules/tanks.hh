#pragma once
#include <ariadne.h>

namespace Flow
{
//!< alpha1_i_val : uscita z_i; beta_i_val : ingresso z_i
HybridIOAutomaton getSystem(
	RealVariable z1, 
	RealVariable z2, 
	RealVariable z3, 
	RealVariable a12, 
	RealVariable a13, 
	RealVariable a23, 
	RealVariable a21, 
	RealVariable a32, 
	RealVariable a31, 
	RealParameter alpha12, 
	RealParameter alpha23, 
	RealParameter alpha32, 
	RealParameter alpha21, 
	RealParameter alpha13, 
	RealParameter alpha31, 
	DiscreteLocation flow)
{
   // Automaton registration
	HybridIOAutomaton hanoi("hanoi");

	//Registration of variables
	hanoi.add_output_var(z1);
	hanoi.add_output_var(z2);
	hanoi.add_output_var(z3);
	hanoi.add_input_var(a12);
	hanoi.add_input_var(a13);
	hanoi.add_input_var(a23);
	hanoi.add_input_var(a21);
	hanoi.add_input_var(a32);
	hanoi.add_input_var(a31);

	//time fixed to first position
	//RealVariable time("0");
	// hanoi.add_output_var(time);

	//Registration of events
	//NO events

	//Registration of locations
	hanoi.new_mode(flow);

	//Registration of dynamics
	hanoi.set_dynamics(flow, z1, - alpha12*a12*z1 + alpha21*a21*z2 - alpha13*a13*z1 + alpha31*a31*z3);
	hanoi.set_dynamics(flow, z2, - alpha21*a21*z2 + alpha12*a12*z1 - alpha23*a23*z2 + alpha32*a32*z3);
	hanoi.set_dynamics(flow, z3, - alpha31*a31*z3 + alpha13*a13*z2 - alpha32*a32*z3 + alpha23*a23*z2);
	// hanoi.set_dynamics(flow, time, 1);

    return hanoi;
}
}
