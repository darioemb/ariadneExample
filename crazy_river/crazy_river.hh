/**
 * @author: Nicola Dessi'
 */
#pragma once
#include<ariadne.h>

HybridIOAutomaton getSystem()
{
	//System variables
	RealVariable a1("a1");
	RealVariable z1("z1");
	RealVariable a2("a2");
	RealVariable z2("z2");

	//System parameters
	RealParameter beta1("beta1",Interval(0.3,0.32863));
	RealParameter beta2("beta2", Interval(0.3,0.32863));
	RealParameter w1("w1", 0.02);
	RealParameter w2("w2", 0.02);

	RealParameter S01("S01", 0.5);
	RealParameter S02("S02", 0.5);
	RealParameter S1("S1", 6.28);
	RealParameter S2("S2", 6.28);

	RealParameter H1("H1", 7.5);
	RealParameter H2("H2", 7.5);

	//Automaton
	HybridIOAutomaton crazy_river("crazy_river");

	//input/output variables
	crazy_river.add_input_var(a1);
	crazy_river.add_input_var(a2);
	crazy_river.add_output_var(z1);
	crazy_river.add_output_var(z2);

	//Events
	DiscreteEvent e_overflow("e_overflow"); //!< event clock
	crazy_river.add_output_event(e_overflow);
	DiscreteEvent e_no_overflow("e_no_overflow");
	crazy_river.add_output_event(e_no_overflow);

	//Locations
	DiscreteLocation no_overflow("no_overflow");
	DiscreteLocation overflow("overflow");

	crazy_river.new_mode(no_overflow);
	crazy_river.new_mode(overflow);

	//dynamics
	crazy_river.set_dynamics(no_overflow, z1, (1.0f/S1)*((-S01*w1*z1)+(beta1*a1)));
	crazy_river.set_dynamics(no_overflow, z2, (1.0f/S2)*((-S02*w2*z2)+(beta2*a2)));
	crazy_river.set_dynamics(overflow, z1, -a1);
	crazy_river.set_dynamics(overflow, z2, (1.0f/S2)*((-S02*w2*z2)+(beta2*a2+beta1*a1-S01*w1*z1)));

	//guards
	RealExpression guard1 = z1-H1; //!< z>=H1
	RealExpression guard2 = -(beta1*a1-S01*w1*z1); //!< z<=beta*a

	std::map<RealVariable, RealExpression> reset1;
	reset1[z1]=z1;
	reset1[z2]=z2;
	std::map<RealVariable, RealExpression> reset2;
	reset2[z2]=z2;

	crazy_river.new_forced_transition(e_overflow, no_overflow, overflow, reset1, guard1);
	crazy_river.new_forced_transition(e_no_overflow, overflow, no_overflow, reset1, guard2);

	return crazy_river;

}
