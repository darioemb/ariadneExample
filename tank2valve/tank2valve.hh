#pragma once
#include <ariadne.h>

namespace Ariadne
{
HybridIOAutomaton getSystem()
{
    RealVariable a("a"); //!< In Valve magnitude variable
    RealVariable z("z"); //!< Water tank variable
    RealVariable w("w"); //!< Out Valve magnitude variable

    RealParameter alpha("alpha", 0.02);                 //!< Coefficient for out water tank
    RealParameter beta("beta", Interval(0.3, 0.32863)); //!< Coefficient for in water tank
    RealParameter T("T", 4.0);
    RealParameter hmin("hmin", 5.75);  // Lower threshold
    RealParameter hmax("hmax", 7.75);  // Upper threshold
    RealParameter delta("delta", 0.1); // Indetermination constant

    //-------- Water tank --------
    // 1.Automaton registration
    HybridIOAutomaton tank("tank");

    // 2.Registration of input/output
    tank.add_input_var(a);
    tank.add_output_var(z);
    // 4.Registration of locations
    DiscreteLocation flow("flow");
    //            DiscreteLocation stormy("rainy");
    tank.new_mode(flow);
    //            tank.new_mode(stormy);

    // 5.Registration of dynamics
    tank.set_dynamics(flow, z, -alpha * w * z + beta * a);

    // 6.Registration of dynamics

    //-------- Input valve --------
    // 1.Automaton registration
    HybridIOAutomaton i_valve("i_valve");

    // 2.Registration of input/output
    i_valve.add_output_var(a);

    // 3.Registration of input/output internal events
    DiscreteEvent e_i_open("i_open");
    DiscreteEvent e_i_close("i_close");
    DiscreteEvent e_i_idle("i_idle");

    i_valve.add_input_event(e_i_open);
    i_valve.add_input_event(e_i_close);
    i_valve.add_internal_event(e_i_idle);

    // 4.Registration of locations
    DiscreteLocation i_opening("i_opening");
    DiscreteLocation i_idle("i_idle");
    DiscreteLocation i_closing("i_closing");

    i_valve.new_mode(i_opening);
    i_valve.new_mode(i_idle);
    i_valve.new_mode(i_closing);

    // 5.Registration of dynamics
    i_valve.set_dynamics(i_idle, a, 0.0);
    i_valve.set_dynamics(i_closing, a, -1.0 / T);
    i_valve.set_dynamics(i_opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    i_valve.new_forced_transition(e_i_idle, i_opening, i_idle, rst_a_one, a_geq_one);
    i_valve.new_forced_transition(e_i_idle, i_closing, i_idle, rst_a_zero, a_leq_zero);

    i_valve.new_unforced_transition(e_i_open, i_idle, i_opening);
    i_valve.new_unforced_transition(e_i_close, i_idle, i_closing);

    //-------- output valve --------
    // 1.Automaton registration
    HybridIOAutomaton o_valve("o_valve");

    // 2.Registration of input/output
    o_valve.add_output_var(w);

    // 3.Registration of input/output internal events
    DiscreteEvent e_o_open("o_open");
    DiscreteEvent e_o_close("o_close");
    DiscreteEvent e_o_idle("o_idle");

    o_valve.add_input_event(e_o_open);
    o_valve.add_input_event(e_o_close);
    o_valve.add_internal_event(e_o_idle);

    // 4.Registration of locations
    DiscreteLocation o_opening("o_opening");
    DiscreteLocation o_idle("o_idle");
    DiscreteLocation o_closing("o_closing");

    o_valve.new_mode(o_opening);
    o_valve.new_mode(o_idle);
    o_valve.new_mode(o_closing);

    // 5.Registration of dynamics
    o_valve.set_dynamics(o_idle, w, 0.0);
    o_valve.set_dynamics(o_closing, w, -1.0 / T);
    o_valve.set_dynamics(o_opening, w, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression w_geq_one = w - 1.0; // w >= 1
    RealExpression w_leq_zero = -w;     // w >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_w_one;
    rst_w_one[w] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_w_zero;
    rst_w_zero[w] = 0.0; // a = 0

    o_valve.new_forced_transition(e_o_idle, o_opening, o_idle, rst_w_zero, w_geq_one);
    o_valve.new_forced_transition(e_o_idle, o_closing, o_idle, rst_w_one, w_leq_zero);

    o_valve.new_unforced_transition(e_o_open, o_idle, o_opening);
    o_valve.new_unforced_transition(e_o_close, o_idle, o_closing);

    //-------- input valve controller --------
    // 1.Automaton
    HybridIOAutomaton controller_i_valve("controller_i_valve");

    // 2.Registration of the input/output variables
    controller_i_valve.add_input_var(z);

    // 3.Registration of the events
    controller_i_valve.add_output_event(e_i_open);
    controller_i_valve.add_output_event(e_i_close);

    // 4.Registration of the locations
    DiscreteLocation i_rising("i_rising");
    DiscreteLocation i_falling("i_falling");

    controller_i_valve.new_mode(i_rising);
    controller_i_valve.new_mode(i_falling);

    // 5.Transitions
    // Invariants
    RealExpression z_leq_hmax = z - hmax - delta; // x <= hmax + delta
    RealExpression z_geq_hmin = hmin - delta - z; // x >= hmin - delta

    controller_i_valve.new_invariant(i_rising, z_leq_hmax);
    controller_i_valve.new_invariant(i_falling, z_geq_hmin);

    // Guards
    RealExpression z_geq_hmax = z - hmax + delta; // x >= hmax - delta
    RealExpression z_leq_hmin = hmin + delta - z; // x <= hmin + delta

    controller_i_valve.new_unforced_transition(e_i_close, i_rising, i_falling, z_geq_hmax);
    controller_i_valve.new_unforced_transition(e_i_open, i_falling, i_rising, z_leq_hmin);

    //-------- output valve controller --------
    // 1.Automaton
    HybridIOAutomaton controller_o_valve("controller_o_valve");

    // 2.Registration of the input/output variables
    controller_o_valve.add_input_var(z);

    // 3.Registration of the events
    controller_o_valve.add_output_event(e_o_open);
    controller_o_valve.add_output_event(e_o_close);

    // 4.Registration of the locations
    DiscreteLocation o_rising("o_rising");
    DiscreteLocation o_falling("o_falling");

    controller_o_valve.new_mode(o_rising);
    controller_o_valve.new_mode(o_falling);

    // 5.Transitions
    // Invariants the same as before

    controller_o_valve.new_invariant(o_rising, z_leq_hmax);
    controller_o_valve.new_invariant(o_falling, z_geq_hmin);

    // Guards same as before

    controller_o_valve.new_unforced_transition(e_o_close, o_falling, o_rising, z_geq_hmax);
    controller_o_valve.new_unforced_transition(e_o_open, o_rising, o_falling, z_leq_hmin);

    // Composition

    HybridIOAutomaton tank_valve = compose("tank,i_valve", tank, i_valve, flow, i_idle);
    HybridIOAutomaton tank_2valve = compose("tank,i_valve,o_valve", tank_valve, o_valve, DiscreteLocation("flow,i_idle"), o_idle);
    HybridIOAutomaton tank_2valve_ci = compose("tank,i_valve,o_valve,controller_i_valve", tank_2valve, controller_i_valve, DiscreteLocation("flow,i_idle,o_idle"), i_rising);
    HybridIOAutomaton system = compose("tank2valve", tank_2valve_ci, controller_o_valve, DiscreteLocation("flow,i_idle,o_idle,i_rising"), o_rising);
    return system;
}
}
