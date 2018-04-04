#pragma once
#include <ariadne.h>

namespace Valve12
{
HybridIOAutomaton getSystem(
	std::string name, 
	RealVariable a, 
	RealParameter T, 
	DiscreteEvent e_open_c, //opening
    DiscreteEvent e_open_h, //opening
	DiscreteEvent e_close_a,//closing
	DiscreteEvent e_close_d,//closing
    DiscreteEvent e_close_i,//closing
    DiscreteEvent e_idle, 
	DiscreteLocation idle,
	DiscreteLocation opening,
    DiscreteLocation closing
	){
	// 1.Automaton registration
    HybridIOAutomaton valve(name);

    // 2.Registration of input/output
    valve.add_output_var(a);

    // 3.Registration of input/output internal events
    valve.add_input_event(e_open_c);
    valve.add_input_event(e_open_h);
    valve.add_input_event(e_close_a);
    valve.add_input_event(e_close_d);
    valve.add_input_event(e_close_i);
    valve.add_internal_event(e_idle);

    // 4.Registration of locations
    valve.new_mode(opening);
    valve.new_mode(idle);
    valve.new_mode(closing);

    // 5.Registration of dynamics
    valve.set_dynamics(idle, a, 0.0);
    valve.set_dynamics(closing, a, -1.0 / T);
    valve.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    valve.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    valve.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    valve.new_unforced_transition(e_open_c, idle, opening);
    valve.new_unforced_transition(e_open_h, idle, opening);
    valve.new_unforced_transition(e_close_a, idle, closing);
    valve.new_unforced_transition(e_close_d, idle, closing);
    valve.new_unforced_transition(e_close_i, idle, closing);

    return valve;	
}
}
namespace Valve13
{
HybridIOAutomaton getSystem(
	std::string name, 
	RealVariable a, 
	RealParameter T, 
	DiscreteEvent e_open_a,
    DiscreteEvent e_open_i,
	DiscreteEvent e_close_b,
    DiscreteEvent e_close_l, 
	DiscreteEvent e_idle, 
	DiscreteLocation idle,
	DiscreteLocation opening,
    DiscreteLocation closing
	)
	{
	// 1.Automaton registration
    HybridIOAutomaton valve(name);

    // 2.Registration of input/output
    valve.add_output_var(a);

    // 3.Registration of input/output internal events
    valve.add_input_event(e_open_a);
    valve.add_input_event(e_open_i);
    valve.add_input_event(e_close_b);
    valve.add_input_event(e_close_l);
    valve.add_internal_event(e_idle);

    // 4.Registration of locations

    valve.new_mode(opening);
    valve.new_mode(idle);
    valve.new_mode(closing);

    // 5.Registration of dynamics
    valve.set_dynamics(idle, a, 0.0);
    valve.set_dynamics(closing, a, -1.0 / T);
    valve.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    valve.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    valve.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    valve.new_unforced_transition(e_open_a, idle, opening);
    valve.new_unforced_transition(e_open_i, idle, opening);
    valve.new_unforced_transition(e_close_b, idle, closing);
    valve.new_unforced_transition(e_close_l, idle, closing);
    	
    return valve;
}
}
namespace Valve21
{
HybridIOAutomaton getSystem(
	std::string name, 
	RealVariable a, 
	RealParameter T, 
	DiscreteEvent e_open_f,
	DiscreteEvent e_close_g, 
	DiscreteEvent e_idle, 
	DiscreteLocation idle,
	DiscreteLocation opening,
    DiscreteLocation closing
	){
	// 1.Automaton registration
    HybridIOAutomaton valve(name);

    // 2.Registration of input/output
    valve.add_output_var(a);

    // 3.Registration of input/output internal events

    valve.add_input_event(e_open_f);
    valve.add_input_event(e_close_g);
    valve.add_internal_event(e_idle);

    // 4.Registration of locations
    valve.new_mode(opening);
    valve.new_mode(idle);
    valve.new_mode(closing);

    // 5.Registration of dynamics
    valve.set_dynamics(idle, a, 0.0);
    valve.set_dynamics(closing, a, -1.0 / T);
    valve.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    valve.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    valve.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    valve.new_unforced_transition(e_open_f, idle, opening);
    valve.new_unforced_transition(e_close_g, idle, closing);	

    return valve;
}
}
namespace Valve23
{
HybridIOAutomaton getSystem(
	std::string name, 
	RealVariable a, 
	RealParameter T, 
	DiscreteEvent e_open_b,
    DiscreteEvent e_open_g,
    DiscreteEvent e_open_l,
	DiscreteEvent e_close_c,
    DiscreteEvent e_close_h,
    DiscreteEvent e_close_m, 
	DiscreteEvent e_idle, 
	DiscreteLocation idle,
	DiscreteLocation opening,
    DiscreteLocation closing
	){
	// 1.Automaton registration
    HybridIOAutomaton valve(name);

    // 2.Registration of input/output
    valve.add_output_var(a);

    // 3.Registration of input/output internal events

    valve.add_input_event(e_open_b);
    valve.add_input_event(e_open_g);
    valve.add_input_event(e_open_l);
    valve.add_input_event(e_close_c);
    valve.add_input_event(e_close_h);
    valve.add_input_event(e_close_m);
    valve.add_internal_event(e_idle);

    // 4.Registration of locations

    valve.new_mode(opening);
    valve.new_mode(idle);
    valve.new_mode(closing);

    // 5.Registration of dynamics
    valve.set_dynamics(idle, a, 0.0);
    valve.set_dynamics(closing, a, -1.0 / T);
    valve.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    valve.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    valve.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    valve.new_unforced_transition(e_open_b, idle, opening);
    valve.new_unforced_transition(e_open_g, idle, opening);
    valve.new_unforced_transition(e_open_l, idle, opening);

    valve.new_unforced_transition(e_close_c, idle, closing);	
    valve.new_unforced_transition(e_close_h, idle, closing);	
    valve.new_unforced_transition(e_close_m, idle, closing);	

    return valve;
}
}
namespace Valve32
{
HybridIOAutomaton getSystem(
	std::string name, 
	RealVariable a, 
	RealParameter T, 
	DiscreteEvent e_open_d,
	DiscreteEvent e_close_e, 
	DiscreteEvent e_idle, 
	DiscreteLocation idle,
	DiscreteLocation opening,
    DiscreteLocation closing
	){
	// 1.Automaton registration
    HybridIOAutomaton valve(name);

    // 2.Registration of input/output
    valve.add_output_var(a);

    // 3.Registration of input/output internal events

    valve.add_input_event(e_open_d);
    valve.add_input_event(e_close_e);
    valve.add_internal_event(e_idle);

    // 4.Registration of locations

    valve.new_mode(opening);
    valve.new_mode(idle);
    valve.new_mode(closing);

    // 5.Registration of dynamics
    valve.set_dynamics(idle, a, 0.0);
    valve.set_dynamics(closing, a, -1.0 / T);
    valve.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    valve.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    valve.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    valve.new_unforced_transition(e_open_d, idle, opening);
    valve.new_unforced_transition(e_close_e, idle, closing);	

    return valve;
}
}
namespace Valve31
{
HybridIOAutomaton getSystem(
	std::string name, 
	RealVariable a, 
	RealParameter T, 
	DiscreteEvent e_open_e,
	DiscreteEvent e_close_f, 
	DiscreteEvent e_idle, 
	DiscreteLocation idle,
	DiscreteLocation opening,
    DiscreteLocation closing
	){
	// 1.Automaton registration
    HybridIOAutomaton valve(name);

    // 2.Registration of input/output
    valve.add_output_var(a);

    // 3.Registration of input/output internal events

    valve.add_input_event(e_open_e);
    valve.add_input_event(e_close_f);
    valve.add_internal_event(e_idle);

    // 4.Registration of locations
    valve.new_mode(opening);
    valve.new_mode(idle);
    valve.new_mode(closing);

    // 5.Registration of dynamics
    valve.set_dynamics(idle, a, 0.0);
    valve.set_dynamics(closing, a, -1.0 / T);
    valve.set_dynamics(opening, a, 1.0 / T);

    // 6.Registration of transitions
    // Guards
    RealExpression a_geq_one = a - 1.0; // a >= 1
    RealExpression a_leq_zero = -a;     // a >= 0

    // Resets
    std::map<RealVariable, RealExpression> rst_a_one;
    rst_a_one[a] = 1.0; // a = 1
    std::map<RealVariable, RealExpression> rst_a_zero;
    rst_a_zero[a] = 0.0; // a = 0

    valve.new_forced_transition(e_idle, opening, idle, rst_a_one, a_geq_one);
    valve.new_forced_transition(e_idle, closing, idle, rst_a_zero, a_leq_zero);

    valve.new_unforced_transition(e_open_e, idle, opening);
    valve.new_unforced_transition(e_close_f, idle, closing);	

    return valve;
}
}