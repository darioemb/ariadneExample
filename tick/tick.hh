/**
 * 	tick example with ariadne
 */
#pragma once
#include<ariadne.h>

HybridIOAutomaton getSystem()
{
	// System variables
	RealVariable x("x"); //tick

	// System parameters
	RealParameter T("T", 1.0); //a clock each second

	HybridIOAutomaton tick("tick");//!< automaton

	tick.add_internal_var(x); //!< is only a self loop, no other systems

	DiscreteEvent clock("clock"); //!< event clock
	tick.add_output_event(clock);

	DiscreteLocation timer("timer"); //!< timer location
	tick.new_mode(timer);

	RealExpression d_x=1; //!< derivate of x is 1

	tick.set_dynamics(timer, x, d_x);
	
	RealExpression guard = x-T; //!< x>T
	std::map<RealVariable, RealExpression> reset;
	reset[x]=0;

	tick.new_forced_transition(clock, timer, timer, reset, guard);

	return tick;
}
