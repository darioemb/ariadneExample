#pragma once
#include <ariadne.h>

namespace CrazyRiver
{
HybridIOAutomaton getSystem(
    RealVariable a,
    RealVariable z1,
    RealVariable z2,
    RealVariable z4,
    RealParameter beta1,
    RealParameter gamma1,
    RealParameter gamma2,
    RealParameter T,
    RealParameter delta,
    RealParameter H1,
    RealParameter H2,
    DiscreteLocation no_overflow)
{
    // 1.Automaton registration
    HybridIOAutomaton crazy_river("crazy_river");

    //tmp
    RealVariable time("0");

    // 2.Registration of input/output variables
    crazy_river.add_input_var(a);
    crazy_river.add_output_var(z1);
    crazy_river.add_output_var(z2);
    crazy_river.add_output_var(z4);
    crazy_river.add_output_var(time);

    // 3.Registration of Events
    DiscreteEvent z1_geq_H1("z1_geq_H1");
    crazy_river.add_internal_event(z1_geq_H1);
    DiscreteEvent z2_geq_H2("z2_geq_H2");
    crazy_river.add_internal_event(z2_geq_H2);
    DiscreteEvent ended_overflow1("ended_overflow1");
    crazy_river.add_internal_event(ended_overflow1);
    DiscreteEvent ended_overflow2("ended_overflow2");
    crazy_river.add_internal_event(ended_overflow2);

    // 4.Registration of Locations
    DiscreteLocation overflow_1("overflow_1");
    DiscreteLocation overflow_2("overflow_2");
    DiscreteLocation overflow_12("overflow_12");

    crazy_river.new_mode(no_overflow); //!< overflow nowhere
    crazy_river.new_mode(overflow_1); //!< overflow on tank 1
    crazy_river.new_mode(overflow_2); //!< overflow on tank 2
    crazy_river.new_mode(overflow_12); //!< overflow on tank 1,2

    // 5.Registration of dynamics
    //no one overflow
    crazy_river.set_dynamics(no_overflow, z1, -gamma1 * z1 + beta1 * a * z4);
    crazy_river.set_dynamics(no_overflow, z2, -gamma2 * z2);
    crazy_river.set_dynamics(no_overflow, z4, gamma1 * z1 + gamma2 * z2 - beta1 * a * z4);
    crazy_river.set_dynamics(no_overflow, time, 1);
    //overflow 1
    crazy_river.set_dynamics(overflow_1, z1, 0);
    crazy_river.set_dynamics(overflow_1, z2, -gamma2 * z2 + beta1 * a * z4 - gamma1 * z1);
    crazy_river.set_dynamics(overflow_1, z4, gamma1 * z1 + gamma2 * z2 - beta1 * a * z4);
    crazy_river.set_dynamics(overflow_1, time, 1);
    //overflow 2
    crazy_river.set_dynamics(overflow_2, z1, beta1 * a * z4 - gamma1 * z1);
    crazy_river.set_dynamics(overflow_2, z2, 0);
    crazy_river.set_dynamics(overflow_2, z4, gamma1 * z1 + gamma2 * z2 - beta1 * a * z4);
    crazy_river.set_dynamics(overflow_2, time, 1);

    //overflow 1,2
    crazy_river.set_dynamics(overflow_12, z1, 0);
    crazy_river.set_dynamics(overflow_12, z2, 0);
    crazy_river.set_dynamics(overflow_12, z4, gamma1 * z1 + gamma2 * z2 - beta1 * a * z4);
    crazy_river.set_dynamics(overflow_12, time, 1);

    //guards
    RealExpression guard_z1_geq_H1 = z1 - H1; //!< z>=H
    RealExpression guard_ended_overflow1 = -(beta1 * a * z4 - gamma1 * z1);
    RealExpression guard_z2_geq_H2 = z2 - H2; //!< z>=H
    RealExpression guard_ended_overflow2 = -((beta1 * a * z4 - gamma1 * z1) - gamma2 * z2);

    std::map<RealVariable, RealExpression> reset;
    reset[z1] = z1;
    reset[z2] = z2;
    reset[z4] = z4;
    reset[time] = time;

    crazy_river.new_forced_transition(z1_geq_H1, no_overflow, overflow_1, reset, guard_z1_geq_H1);
    crazy_river.new_forced_transition(z2_geq_H2, no_overflow, overflow_2, reset, guard_z2_geq_H2);

    crazy_river.new_forced_transition(z2_geq_H2, overflow_1, overflow_12, reset, guard_z2_geq_H2);
    crazy_river.new_forced_transition(ended_overflow1, overflow_1, no_overflow, reset, guard_ended_overflow1);

    crazy_river.new_forced_transition(z1_geq_H1, overflow_2, overflow_12, reset, guard_z1_geq_H1);
    crazy_river.new_forced_transition(ended_overflow2, overflow_2, no_overflow, reset, guard_ended_overflow2);

    crazy_river.new_forced_transition(ended_overflow1, overflow_12, overflow_2, reset, guard_ended_overflow1);
    crazy_river.new_forced_transition(ended_overflow2, overflow_12, overflow_1, reset, guard_ended_overflow2);

    return crazy_river;
}
}
