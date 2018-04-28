#pragma once
#include <ariadne.h>

namespace controller
{
HybridIOAutomaton getSystem(
    RealVariable z1,
    RealVariable z2,
    RealVariable a1,
    RealVariable a2,
    RealParameter baseLevel1,
    RealParameter targetLevel1,
    RealParameter baseLevel2,
    RealParameter targetLevel2,
    RealParameter delta,
    DiscreteEvent e_a1_open,
    DiscreteEvent e_a1_close,
    DiscreteEvent e_a2_open,
    DiscreteEvent e_a2_close,
    DiscreteLocation on_first)
{
  HybridIOAutomaton controller("controller");

  DiscreteLocation out_lock("out_lock");

  controller.add_output_event(e_a1_open);
  controller.add_output_event(e_a1_close);
  controller.add_output_event(e_a2_open);
  controller.add_output_event(e_a2_close);

  controller.add_input_var(z1);

  DiscreteLocation waiting_on("waiting_on");
  DiscreteLocation on_second("on_second");

  controller.new_mode(on_first);
  controller.new_mode(waiting_on);
  controller.new_mode(on_second);
  controller.new_mode(out_lock);

  RealParameter epsilon("epsilon",0.5);

  RealExpression z1_geq_targetLevel1 = z1 - targetLevel1+epsilon + delta;
  RealExpression z1_leq_baseLevel1 = baseLevel1 - z1 + delta;
  RealExpression z2_geq_targetLevel2 = z2 - targetLevel2+epsilon + delta;
  RealExpression z2_leq_baseLevel2 = baseLevel2 - z2 + delta;

  controller.new_forced_transition(e_a1_close, on_first, waiting_on, z1_geq_targetLevel1);
  controller.new_forced_transition(e_a2_open, waiting_on, on_second, z1_leq_baseLevel1);
  controller.new_forced_transition(e_a2_close, on_second, out_lock, z2_geq_targetLevel2);
  controller.new_forced_transition(e_a1_open, out_lock, on_first, z2_leq_baseLevel2);

  return controller;
}
}
