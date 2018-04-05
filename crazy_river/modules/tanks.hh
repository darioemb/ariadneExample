#pragma once
#include <ariadne.h>

namespace CrazyRiver
{
HybridIOAutomaton getSystem(
    RealVariable a,
    RealVariable z1,
    RealVariable z2,
    RealVariable z4,
    RealParameter alpha1,
    RealParameter beta1,
    RealParameter beta2,
    RealParameter gamma1,
    RealParameter gamma2,
    RealParameter T,
    RealParameter hmin,
    RealParameter hmax,
    RealParameter delta,
    RealParameter H1,
    RealParameter H2,
    DiscreteLocation S0)
{
    // 1.Automaton registration
    HybridIOAutomaton crazy_river("crazy_river");

    //tmp
    // RealVariable time("0");

    // 2.Registration of input/output variables
    crazy_river.add_input_var(a);
    crazy_river.add_output_var(z1);
    crazy_river.add_output_var(z2);
    crazy_river.add_output_var(z4);
    // crazy_river.add_output_var(time);

    // 3.Registration of Events
    DiscreteEvent e_a("e_a");
    crazy_river.add_output_event(e_a);
    DiscreteEvent e_b("e_b");
    crazy_river.add_output_event(e_b);
    DiscreteEvent e_d("e_d");
    crazy_river.add_output_event(e_d);
    DiscreteEvent e_e("e_e");
    crazy_river.add_output_event(e_e);

    // 4.Registration of Locations
    DiscreteLocation S1("S1");
    DiscreteLocation S2("S2");
    DiscreteLocation S3("S3");

    crazy_river.new_mode(S0); //!< overflow nowhere
    crazy_river.new_mode(S1); //!< overflow on tank 1
    crazy_river.new_mode(S2); //!< overflow on tank 2
    crazy_river.new_mode(S3); //!< overflow on tank 1,2

    // 5.Registration of dynamics
    //no one overflow
    crazy_river.set_dynamics(S0, z1, -alpha1 * z1 - gamma1 * z1 + beta1 * (a/2) * z4);
    crazy_river.set_dynamics(S0, z2, alpha1*z1 - gamma2 * z2 + beta2 * (a/2) * z4);
    crazy_river.set_dynamics(S0, z4, gamma1 * z1 + gamma2 * z2 - beta1 * (a/2) * z4 - beta2 * (a/2) * z4);
    // crazy_river.set_dynamics(S0, time, 1);
    //overflow 1
    crazy_river.set_dynamics(S1, z1, 0);
    crazy_river.set_dynamics(S1, z2, -gamma2 * z2 + beta2 * (a/2) * z4 + (beta1 * (a/2) * z4 - alpha1 * z1));
    crazy_river.set_dynamics(S1, z4, gamma1 * z1 + gamma2 * z2 - beta1 * (a/2) * z4 - beta2 * (a/2) * z4);
    // crazy_river.set_dynamics(S1, time, 1);
    //overflow 2
    crazy_river.set_dynamics(S2, z1, -alpha1 * z1 + beta1 * (a/2) * z4 - gamma1 * z1);
    crazy_river.set_dynamics(S2, z2, 0);
    crazy_river.set_dynamics(S2, z4, gamma1 * z1 + gamma2 * z2 - beta1 * (a/2) * z4 - beta2 * (a/2) * z4);
    // crazy_river.set_dynamics(S2, time, 1);

    //overflow 1,2
    crazy_river.set_dynamics(S3, z1, 0);
    crazy_river.set_dynamics(S3, z2, 0);
    crazy_river.set_dynamics(S3, z4, gamma1 * z1 + gamma2 * z2 - beta1 * (a/2) * z4 - beta2 * (a/2) * z4);
    // crazy_river.set_dynamics(S3, time, 1);

    //guards
    RealExpression guard_a = z1 - H1;      //!< z>=H
    // RealExpression guard_d = -z1+H1;   
    RealExpression guard_d = -beta1 * (a/2) * z4 - alpha1 * z1 - gamma1 * z1; //!< z<=beta*a
    RealExpression guard_b = z2 - H2;                                     //!< z>=H
    // RealExpression guard_e = -z2+H2;
    RealExpression guard_e = -beta2 * (a/2) * z4  - gamma2 * z2; //!< z<=beta*a

    std::map<RealVariable, RealExpression> reset;
    reset[z1] = z1;
    reset[z2] = z2;
    reset[z4] = z4;
    // // reset[time] = time;

    /**
         * 			TRANSITION TABLE
         * ---------------------
         * 	 |	S0	S1	S2	S3	
         * ---------------------
         * a |	S1	-	S3	-	
         * b |	S2	S3	-	-	
         * d |	-	S0	-	S2	
         * e |	-	-	S0	S1	
         */
    crazy_river.new_forced_transition(e_a, S0, S1, reset, guard_a);
    crazy_river.new_forced_transition(e_b, S0, S2, reset, guard_b);

    crazy_river.new_forced_transition(e_b, S1, S3, reset, guard_b);
    crazy_river.new_forced_transition(e_d, S1, S0, reset, guard_d);

    crazy_river.new_forced_transition(e_a, S2, S3, reset, guard_a);
    crazy_river.new_forced_transition(e_e, S2, S0, reset, guard_e);

    crazy_river.new_forced_transition(e_d, S3, S2, reset, guard_d);
    crazy_river.new_forced_transition(e_e, S3, S1, reset, guard_e);

    return crazy_river;
}
}
