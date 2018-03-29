#pragma once
#include <ariadne.h>

namespace CrazyRiver
{
//!< alpha1_i_val : uscita z_i; beta_i_val : ingresso z_i
HybridIOAutomaton getSystem(RealVariable a, RealVariable z1, RealVariable z2, RealVariable z3, RealVariable z4, RealParameter alpha1, RealParameter alpha2, RealParameter alpha3, RealParameter beta1, RealParameter beta2, RealParameter beta3, RealParameter T, RealParameter hmin, RealParameter hmax, RealParameter delta, RealParameter H1, RealParameter H2, RealParameter H3, DiscreteLocation S0)
{
    // 1.Automaton registration
    HybridIOAutomaton crazy_river("crazy_river");

    // 2.Registration of input/output variables
    crazy_river.add_input_var(a);
    crazy_river.add_output_var(z1);
    crazy_river.add_output_var(z2);
    crazy_river.add_output_var(z3);
    crazy_river.add_output_var(z4);

    // 3.Registration of Events
    DiscreteEvent e_a("e_a");
    crazy_river.add_output_event(e_a);
    DiscreteEvent e_b("e_b");
    crazy_river.add_output_event(e_b);
    DiscreteEvent e_c("e_c");
    crazy_river.add_output_event(e_c);
    DiscreteEvent e_d("e_d");
    crazy_river.add_output_event(e_d);
    DiscreteEvent e_e("e_e");
    crazy_river.add_output_event(e_e);
    DiscreteEvent e_f("e_f");
    crazy_river.add_output_event(e_f);

    // 4.Registration of Locations
    DiscreteLocation S1("S1");
    DiscreteLocation S2("S2");
    DiscreteLocation S3("S3");
    DiscreteLocation S4("S4");
    DiscreteLocation S5("S5");
    DiscreteLocation S6("S6");
    DiscreteLocation S7("S7");

    crazy_river.new_mode(S0); //!< overflow nowhere
    crazy_river.new_mode(S1); //!< overflow on tank 1
    crazy_river.new_mode(S2); //!< overflow on tank 2
    crazy_river.new_mode(S3); //!< overflow on tank 3
    crazy_river.new_mode(S4); //!< overflow on tank 1,2
    crazy_river.new_mode(S5); //!< overflow on tank 1,3
    crazy_river.new_mode(S6); //!< overflow on tank 2,3
    crazy_river.new_mode(S7); //!< overflow everywhere

    // 5.Registration of dynamics
    //no one overflow
    crazy_river.set_dynamics(S0, z1, -alpha1 * z1 + beta1 * a);
    crazy_river.set_dynamics(S0, z2, -alpha2 * z2 + beta2 * a);
    crazy_river.set_dynamics(S0, z3, -alpha3 * z3 + beta3 * a);
    crazy_river.set_dynamics(S0, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);
    //overflow 1
    crazy_river.set_dynamics(S1, z1, 0);
    crazy_river.set_dynamics(S1, z2, -alpha2 * z2 + beta2 * a + beta1 * a - alpha1 * z1);
    crazy_river.set_dynamics(S1, z3, -alpha3 * z3 + beta3 * a);
    crazy_river.set_dynamics(S1, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);
    //overflow 2
    crazy_river.set_dynamics(S2, z1, -alpha1 * z1 + beta1 * a);
    crazy_river.set_dynamics(S2, z2, 0);
    crazy_river.set_dynamics(S2, z3, -alpha3 * z3 + beta3 * a - alpha2 * z2 + beta2 * a);
    crazy_river.set_dynamics(S2, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);
    //overflow 3
    crazy_river.set_dynamics(S3, z1, -alpha1 * z1 + beta1 * a);
    crazy_river.set_dynamics(S3, z2, -alpha2 * z2 + beta2 * a);
    crazy_river.set_dynamics(S3, z3, 0);
    crazy_river.set_dynamics(S3, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);
    //overflow 1,2
    crazy_river.set_dynamics(S4, z1, 0);
    crazy_river.set_dynamics(S4, z2, 0);
    crazy_river.set_dynamics(S4, z3, -alpha3 * z3 + beta3 * a - alpha2 * z2 + beta2 * a - alpha1 * z1 + beta1 * a);
    crazy_river.set_dynamics(S4, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);
    //overflow 1,3
    crazy_river.set_dynamics(S5, z1, 0);
    crazy_river.set_dynamics(S5, z2, -alpha2 * z2 + beta2 * a + beta1 * a - alpha1 * z1);
    crazy_river.set_dynamics(S5, z3, 0);
    crazy_river.set_dynamics(S5, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);
    //overflow 2,3
    crazy_river.set_dynamics(S6, z1, -alpha1 * z1 + beta1 * a);
    crazy_river.set_dynamics(S6, z2, 0);
    crazy_river.set_dynamics(S6, z3, 0);
    crazy_river.set_dynamics(S6, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);
    //overflow 1,2,3
    crazy_river.set_dynamics(S7, z1, 0);
    crazy_river.set_dynamics(S7, z2, 0);
    crazy_river.set_dynamics(S7, z3, 0);
    crazy_river.set_dynamics(S7, z4, alpha1 * z1 + alpha2 * z2 + alpha3 * z3 - beta1 * a - beta2 * a - beta3 * a);

    //guards
    RealExpression guard_a = z1 - H1;                  //!< z>=H
    RealExpression guard_d = -beta1 * a - alpha1 * z1; //!< z<=beta*a
    RealExpression guard_b = z2 - H2;                  //!< z>=H
    RealExpression guard_e = -beta2 * a - alpha2 * z2; //!< z<=beta*a
    RealExpression guard_c = z3 - H3;                  //!< z>=H
    RealExpression guard_f = -beta3 * a - alpha3 * z3; //!< z<=beta*a

    std::map<RealVariable, RealExpression> reset;
    reset[z1] = z1;
    reset[z2] = z2;
    reset[z3] = z3;
    reset[z4] = z4;

    /**
         * 			TRANSITION TABLE
         * --------------------------------------
         * 	 |	S0	S1	S2	S3	S4	S5	S6	S7
         * --------------------------------------
         * a |	S1	-	S4	S5	-	-	S7	-
         * b |	S2	S4	-	S6	-	S7	-	-
         * c |	S3	S5	S6	-	S7	-	-	-
         * d |	-	S0	-	-	S2	S3	-	S6
         * e |	-	-	S0	-	S1	-	S3	S5
         * f |	-	-	-	S0	-	S1	S2	S4
         */
    crazy_river.new_forced_transition(e_a, S0, S1, reset, guard_a);
    crazy_river.new_forced_transition(e_b, S0, S2, reset, guard_b);
    crazy_river.new_forced_transition(e_c, S0, S3, reset, guard_c);

    crazy_river.new_forced_transition(e_b, S1, S4, reset, guard_b);
    crazy_river.new_forced_transition(e_c, S1, S5, reset, guard_c);
    crazy_river.new_forced_transition(e_d, S1, S0, reset, guard_d);

    crazy_river.new_forced_transition(e_a, S2, S4, reset, guard_a);
    crazy_river.new_forced_transition(e_c, S2, S6, reset, guard_c);
    crazy_river.new_forced_transition(e_e, S2, S0, reset, guard_e);

    crazy_river.new_forced_transition(e_a, S3, S5, reset, guard_a);
    crazy_river.new_forced_transition(e_b, S3, S6, reset, guard_b);
    crazy_river.new_forced_transition(e_f, S3, S0, reset, guard_f);

    crazy_river.new_forced_transition(e_c, S4, S7, reset, guard_c);
    crazy_river.new_forced_transition(e_d, S4, S2, reset, guard_d);
    crazy_river.new_forced_transition(e_e, S4, S1, reset, guard_e);

    crazy_river.new_forced_transition(e_b, S5, S7, reset, guard_b);
    crazy_river.new_forced_transition(e_d, S5, S3, reset, guard_d);
    crazy_river.new_forced_transition(e_f, S5, S1, reset, guard_f);

    crazy_river.new_forced_transition(e_a, S6, S7, reset, guard_a);
    crazy_river.new_forced_transition(e_e, S6, S3, reset, guard_e);
    crazy_river.new_forced_transition(e_f, S6, S2, reset, guard_f);

    crazy_river.new_forced_transition(e_d, S7, S6, reset, guard_d);
    crazy_river.new_forced_transition(e_e, S7, S5, reset, guard_e);
    crazy_river.new_forced_transition(e_f, S7, S4, reset, guard_f);

    return crazy_river;
}
}
