#pragma once
#include <ariadne.h>

namespace controller
{
HybridIOAutomaton getSystem(
	RealVariable z1,
	RealVariable z2,
	RealVariable a1,
	RealVariable a2, 
	RealParameter hmin1, 
	RealParameter hmax1,
	RealParameter hmin2, 
	RealParameter hmax2,
	RealParameter delta, 
	DiscreteEvent e_a1_open, 
	DiscreteEvent e_a1_close, 
	DiscreteEvent e_a2_open, 
	DiscreteEvent e_a2_close, 
	DiscreteLocation on_first
	)
{
  HybridIOAutomaton controller("controller");

  DiscreteLocation on_second("on_second");
  DiscreteLocation out_lock("out_lock");

  controller.add_output_event(e_a1_open);
  controller.add_output_event(e_a1_close);
  controller.add_output_event(e_a2_open);
  controller.add_output_event(e_a2_close);


  controller.add_input_var(z1);
  controller.add_input_var(z2);

  DiscreteLocation transient("closing_a1");

  controller.new_mode(on_first);
  controller.new_mode(on_second);
  controller.new_mode(out_lock);
  controller.new_mode(transient);

  RealExpression z1_leq_hmax1 = z1 - hmax1;
  RealExpression z2_leq_hmax2 = z2 - hmax2;
  RealExpression z2_geq_hmin2 = hmin2 - z2;
  RealExpression z1_geq_hmin1 = hmin1 - z1;

  RealExpression z1_geq_hmax1 = z1 - hmax1;
  RealExpression z2_geq_hmax2 = z2 - hmax2;
  RealExpression z2_leq_hmin2 = hmin2 - z2;
  RealExpression z1_leq_hmin1 = hmin1 - z1;

  //controller.new_invariant(on_first, z1_leq_hmax1);
  //controller.new_invariant(transient, z1_geq_hmin1);
  //controller.new_invariant(on_second, z2_leq_hmax2);
  //controller.new_invariant(out_lock, z2_geq_hmin2);

  controller.new_forced_transition(e_a1_close, on_first, transient, z1_geq_hmax1);
  controller.new_forced_transition(e_a2_open, transient, on_second, z1_leq_hmin1);
  
  controller.new_forced_transition(e_a2_close, on_second, out_lock, z2_geq_hmax2);
  controller.new_forced_transition(e_a2_close, out_lock, on_first, z2_leq_hmin2);
  

    return controller;
}
}
