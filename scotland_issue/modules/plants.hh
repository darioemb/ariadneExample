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
    DiscreteLocation storm,
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
    DiscreteEvent stormy("stormy");
    system.add_output_event(sunny);
    system.add_output_event(stormy);

    // 4.Registration of Locations
    DiscreteLocation sun("sun");
    system.new_mode(storm);
    system.new_mode(sun);

    RealParameter c("c",1.0);

    // 5.Registration of dynamics
    system.set_dynamics(storm, w, c*(w+w*w));
    system.set_dynamics(storm, time, 1);

    system.set_dynamics(sun, w, -c*(w+w*w));
    system.set_dynamics(sun, time, 1);

    std::map<RealVariable, RealExpression> reset;
    reset[w] = w;
    reset[time] = time;

    RealExpression guard_sunny = w - treshold; //!< w>=treshold
    RealExpression guard_stormy = -w+0.5; //!< w>=treshold

    system.new_forced_transition(sunny, storm, sun, reset, guard_sunny);
    system.new_forced_transition(stormy, sun, storm, reset, guard_stormy);

    return system;
}
}