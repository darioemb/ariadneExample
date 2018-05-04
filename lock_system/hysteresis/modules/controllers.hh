#pragma once
#include <ariadne.h>

namespace controller
{
HybridIOAutomaton getSystem(
	RealVariable z,
	RealVariable a,
	RealParameter baseLevel, 
	RealParameter targetLevel,
	RealParameter delta, 
	DiscreteEvent e_a_open, 
	DiscreteEvent e_a_close, 
	DiscreteLocation on_first
	)
{
  HybridIOAutomaton controller("controller");

  DiscreteLocation out_lock("out_lock");

  controller.add_output_event(e_a_open);
  controller.add_output_event(e_a_close);

  controller.add_input_var(z);

  controller.new_mode(on_first);
  controller.new_mode(out_lock);

  RealExpression z_geq_targetLevel = z - targetLevel;
  RealExpression z_leq_baseLevel = baseLevel - z;
  

  controller.new_forced_transition(e_a_close, on_first, out_lock, z_geq_targetLevel);
  controller.new_forced_transition(e_a_open, out_lock, on_first, z_leq_baseLevel);

    return controller;
}
}
