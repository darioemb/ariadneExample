#pragma once
#include <ariadne.h>

namespace scotland_issue
{
HybridIOAutomaton getSystem(
    RealVariable a,
    RealVariable d,
    RealVariable w,
    RealVariable z,
    RealParameter alpha,
    RealParameter beta,
    RealParameter gamma,
    RealParameter H,
    DiscreteLocation no_overflow)
{
    HybridIOAutomaton system("scotland_issue");

    system.add_input_var(a);
    system.add_input_var(w);
    system.add_output_var(z);
    system.add_output_var(d);

    DiscreteEvent z_geq_H("z_geq_H");
    DiscreteEvent ended_overflow("ended_overflow");
    system.add_internal_event(z_geq_H);
    system.add_internal_event(ended_overflow);

    DiscreteLocation overflow("overflow");

    system.new_mode(no_overflow);
    system.new_mode(overflow);

    system.set_dynamics(no_overflow, z, a * beta * d - alpha * z + w * gamma);
    system.set_dynamics(no_overflow, d, -a * beta * d + alpha * z);

    system.set_dynamics(overflow, z, 0);
    system.set_dynamics(overflow, d, -a * beta * d + alpha * z);

    RealExpression guard_z_geq_H = z - H;
    RealExpression guard_ended_overflow = -(a * beta * d + w * gamma - alpha * z);

    std::map<RealVariable, RealExpression> reset;
    reset[z] = z;
    reset[d] = d;

    system.new_forced_transition(z_geq_H, no_overflow, overflow, guard_z_geq_H);
    system.new_forced_transition(ended_overflow, overflow, no_overflow, guard_ended_overflow);

    return system;
}
}

namespace stormy_weather
{
HybridIOAutomaton getSystem(
    RealVariable w,
    DiscreteEvent sunny,
    DiscreteLocation tick,
    RealParameter treshold)
{
    // 1.Automaton registration
    HybridIOAutomaton system("stormy");

    // 2.Registration of input/output variables
    system.add_output_var(w);

    //tmp
    RealVariable time("0");
    system.add_output_var(time);

    // 3.Registration of Events
    system.add_output_event(sunny);

    // 4.Registration of Locations
    system.new_mode(tick);

    RealParameter c("c",0.5);

    // 5.Registration of dynamics
    system.set_dynamics(tick, w, w*c);
    system.set_dynamics(tick, time, 1);

    std::map<RealVariable, RealExpression> reset;
    reset[w] = 0.0001;
    reset[time] = time;

    RealExpression guard_sunny = w - treshold; //!< w>=treshold

    system.new_forced_transition(sunny, tick, tick, reset, guard_sunny);

    return system;
}
}