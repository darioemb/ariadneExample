#pragma once
#include <ariadne.h>

namespace Controller
{
HybridIOAutomaton getSystem(
	RealVariable z1,
	RealVariable z2,
	RealVariable z3,
	DiscreteEvent a,
	DiscreteEvent b,
	DiscreteEvent c,
	DiscreteEvent d,
	DiscreteEvent e,
	DiscreteEvent f,
	DiscreteEvent g,
	DiscreteEvent h,
	DiscreteEvent i,
	DiscreteEvent l,
	DiscreteEvent m,
	RealParameter H1,
	RealParameter H2,
	RealParameter H3,
	DiscreteLocation S0)
{
	// 1.Automaton
	HybridIOAutomaton controller("controller");

	// 2.Registration of the input/output variables
	controller.add_input_var(z1);
	controller.add_input_var(z2);
	controller.add_input_var(z3);

	controller.add_output_event(m);

	// 3.Registration of the events
	DiscreteLocation S1("S1");
	DiscreteLocation S2("S2");
	DiscreteLocation S3("S3");
	DiscreteLocation S4("S4");
	DiscreteLocation S5("S5");
	DiscreteLocation S6("S6");
	DiscreteLocation S7("S7");
	DiscreteLocation S8("S8");
	DiscreteLocation S9("S9");
	DiscreteLocation S10("S10");
	DiscreteLocation S11("S11");

	controller.new_mode(S0);
	controller.new_mode(S1);
	controller.new_mode(S2);
	controller.new_mode(S3);
	controller.new_mode(S4);
	controller.new_mode(S5);
	controller.new_mode(S6);
	controller.new_mode(S7);
	controller.new_mode(S8);
	controller.new_mode(S9);
	controller.new_mode(S10);
	controller.new_mode(S11);

	RealExpression z1_geq_123 = z1- H1 - H2 - H3;
	RealExpression z1_geq_23 = z1 - H2 - H3;
	RealExpression z1_geq_2 = z1 - H2;
	RealExpression z1_geq_3 = z1 - H3;
	RealExpression z2_geq_3 = z2 - H3;
	RealExpression z3_geq_2 = z3 - H2;
	RealExpression z1_geq_0 = z1;
	RealExpression z2_geq_0 = z2;
	RealExpression z3_geq_0 = z3;

	controller.new_invariant(S0, z1_geq_23);
	controller.new_invariant(S1, z1_geq_3);
	controller.new_invariant(S2, z2_geq_0);
	controller.new_invariant(S3, z1_geq_0);
	controller.new_invariant(S4, z3_geq_2);
	controller.new_invariant(S5, z3_geq_0);
	controller.new_invariant(S6, z2_geq_3);
	controller.new_invariant(S7, z2_geq_0);
	controller.new_invariant(S8, z1_geq_2);
	controller.new_invariant(S9, z1_geq_0);
	controller.new_invariant(S10, z2_geq_0);
	controller.new_invariant(S11, z1_geq_123);

	RealExpression z1_l_23 = -z1 + H2 + H3;
	RealExpression z1_l_2 = -z1 + H2;
	RealExpression z1_l_3 = -z1 + H3;
	RealExpression z2_l_3 = -z2 + H3;
	RealExpression z3_l_2 = -z3 + H2;
	RealExpression z1_l_0 = -z1;
	RealExpression z2_l_0 = -z2;
	RealExpression z3_l_0 = -z3;

	controller.new_unforced_transition(a,S0,S1,z1_l_23);
	controller.new_unforced_transition(b,S1,S2,z1_l_3);
	controller.new_unforced_transition(c,S2,S3,z2_l_0);
	controller.new_unforced_transition(d,S3,S4,z1_l_0);
	controller.new_unforced_transition(e,S4,S5,z3_l_2);
	controller.new_unforced_transition(f,S5,S6,z3_l_0);
	controller.new_unforced_transition(g,S6,S7,z2_l_3);
	controller.new_unforced_transition(h,S7,S8,z2_l_0);
	controller.new_unforced_transition(i,S8,S9,z1_l_2);
	controller.new_unforced_transition(l,S9,S10,z1_l_0);
	controller.new_unforced_transition(m,S10,S11,z2_l_0);

	
	return controller;
}
}