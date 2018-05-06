#pragma once
#include <ariadne.h>

namespace controller_hysteresis
{
HybridIOAutomaton getSystem(
	RealVariable z1,
	RealVariable z2,
	RealParameter hmin,
	RealParameter hmax,
	RealParameter delta,
	DiscreteEvent e_open,
	DiscreteEvent e_close,
	DiscreteLocation rising)
{
	// 1.Automaton
	HybridIOAutomaton system("h_controller");

	// 2.Registration of the input/output variables
	system.add_input_var(z1);
	system.add_input_var(z2);

	// 3.Registration of the events
	system.add_output_event(e_open);
	system.add_output_event(e_close);

	// 4.Registration of the locations
	DiscreteLocation falling("falling");

	system.new_mode(rising);
	system.new_mode(falling);

	// 5.Transitions
	// Invariants
	RealExpression z_geq_hmin = (hmin - delta - z1); // x >= hmin - delta
	RealExpression z_leq_hmax = (z2 - hmax - delta); // x <= hmax + delta

	system.new_invariant(rising, z_leq_hmax);
	system.new_invariant(falling, z_geq_hmin);

	// Guards
	RealExpression z_geq_hmax = (z2 - hmax + delta); // z1 >= hmax - delta
	RealExpression z_leq_hmin = (hmin + delta - z1); // z1 <= hmin + delta

	system.new_unforced_transition(e_close, rising, falling, z_geq_hmax);
	system.new_unforced_transition(e_open, falling, rising, z_leq_hmin);

	return system;
}
}

namespace controller_proportional
{
HybridIOAutomaton getSystem(
	RealVariable a,
	RealVariable z,
	RealParameter delta,
	RealParameter Kp,
	RealParameter ref,
	RealParameter tau,
	DiscreteLocation stabilizing
)
{
	HybridIOAutomaton system("p_controller");

	// Add the input/output variables
	system.add_input_var(z);
	system.add_output_var(a);

	// States
	DiscreteLocation opening("opening");
	DiscreteLocation closing("closing");

	// Create the discrete events
	DiscreteEvent stabilise_after_closing("stabilise_after_closing");
	DiscreteEvent stabilise_after_opening("stabilise_after_opening");
	DiscreteEvent close("close");
	DiscreteEvent open("open");

	// Create the dynamics
	RealExpression opening_d = (1 - a) / tau;
	RealExpression closing_d = -a / tau;
	RealExpression stabilizing_d = (Kp * (ref - z) - a) / tau;

	// Dynamics at the different modes
	system.new_mode(opening);
	system.set_dynamics(opening, a, opening_d);
	system.new_mode(closing);
	system.set_dynamics(closing, a, closing_d);
	system.new_mode(stabilizing);
	system.set_dynamics(stabilizing, a, stabilizing_d);

	// Invariants
	RealExpression a_geq_zero = -a;		// a >= 0
	RealExpression a_leq_one = a - 1.0; // a <= 1

	// Create the guards
	RealExpression z_lesser_ref_minus_delta = -z - delta + ref;				  
	RealExpression z_greater_ref_minus_delta = z - delta - ref;				  
	RealExpression z_lesser_ref_kp_minus_delta = -z + ref - 1.0 / Kp - delta; 
	RealExpression z_greater_ref_kp_minus_delta = z - ref + 1.0 / Kp - delta; 

	// Transitions
	system.new_forced_transition(stabilise_after_closing, closing, stabilizing, z_lesser_ref_minus_delta);
	system.new_forced_transition(stabilise_after_opening, opening, stabilizing, z_greater_ref_kp_minus_delta);
	system.new_forced_transition(close, stabilizing, closing, z_greater_ref_minus_delta);
	system.new_forced_transition(open, stabilizing, opening, z_lesser_ref_kp_minus_delta);

	return system;
}
}