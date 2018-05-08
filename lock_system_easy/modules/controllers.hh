#pragma once
#include <ariadne.h>

namespace controller
{
HybridIOAutomaton getSystem(
	RealVariable z,
	RealVariable a,
  RealVariable t,
	RealParameter thr,
  RealParameter kp,
  RealParameter tau,
  RealParameter ref, 
	RealParameter delta, 
	DiscreteEvent z_leq_riverLevel, 
	DiscreteLocation stabilizing)
  



{
  HybridIOAutomaton controller("controller");

  DiscreteLocation opening("opening");
  DiscreteLocation closing("closing");
  DiscreteLocation idle("idle");

  //t try with internal var
  controller.add_output_var(a);
  controller.add_output_var(t);
  controller.add_input_var(z);


  controller.new_mode(opening);
  controller.new_mode(closing);
  controller.new_mode(stabilizing);
  controller.new_mode(idle);



  DiscreteEvent z_geq_ref_min_kp("z_geq_ref_min_kp");
  DiscreteEvent z_leq_ref_min_kp("z_leq_ref_min_kp");
  DiscreteEvent z_geq_ref("z_geq_ref");
  DiscreteEvent z_leq_ref("z_leq_ref");
  DiscreteEvent t_geq_thr("t_geq_thr");

  controller.add_internal_event(z_geq_ref_min_kp);
  controller.add_internal_event(z_leq_ref_min_kp);
  controller.add_internal_event(z_geq_ref);
  controller.add_internal_event(z_leq_ref);
  controller.add_internal_event(t_geq_thr);

  controller.add_input_event(z_leq_riverLevel);

  controller.set_dynamics(opening, a, (1-a)/tau);
  controller.set_dynamics(closing, a, (-a)/tau);
  controller.set_dynamics(stabilizing, a, (kp*(ref-z)-a)/tau);
  controller.set_dynamics(idle, a, 0);

  controller.set_dynamics(opening, t, 1);
  controller.set_dynamics(closing, t, 1);
  controller.set_dynamics(stabilizing, t, 1);
  controller.set_dynamics(idle, t, 0);

  RealExpression guard_z_geq_ref_min_kp=(z-ref+(1.0/kp)-delta);
  RealExpression guard_z_leq_ref_min_kp=-(z-ref+(1.0/kp)+delta);
  RealExpression guard_z_geq_ref=(z-ref-delta);
  RealExpression guard_z_leq_ref=-(z-ref+delta);
  RealExpression guard_t_geq_thr=t-thr;

  std::map<RealVariable, RealExpression> reset;
  reset[a] = a;
  reset[t] = 0.0;



  controller.new_forced_transition(z_geq_ref_min_kp, opening, stabilizing, guard_z_geq_ref_min_kp);
  controller.new_forced_transition(z_leq_ref_min_kp, stabilizing, opening, guard_z_leq_ref_min_kp);
  controller.new_forced_transition(z_geq_ref, stabilizing, closing, guard_z_geq_ref);
  controller.new_forced_transition(z_leq_ref, closing, stabilizing, guard_z_leq_ref);
  controller.new_forced_transition(t_geq_thr, stabilizing, idle, reset, guard_t_geq_thr);
  controller.new_forced_transition(t_geq_thr, opening, idle, reset, guard_t_geq_thr);
  controller.new_forced_transition(t_geq_thr, closing, idle, reset,guard_t_geq_thr);
  

  //maybe unforced
  controller.new_unforced_transition(z_leq_riverLevel, idle, stabilizing);

  return controller;
}
}