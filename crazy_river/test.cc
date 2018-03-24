#include <ariadne.h>
#include "crazy_river.hh"
#include "analysis.hh" // Custom analysis routines to be run

int main(int argc, char **argv)
{
    int verb = 0;
    if (argc > 1)
        verb = atoi(argv[1]);

    bool plot_results = true;

    HybridIOAutomaton system = Ariadne::getSystem(0.01, 0.01, 0.01, 0.02, 0.04,0.01);

    cout << system << endl;

    HybridBoundedConstraintSet initial_set(system.state_space());
    initial_set[DiscreteLocation("S0,idle,rising")] = Box(5, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0, 1.0, 2.0, 1.0, 2.0);

    //initial_set[DiscreteLocation("no_overflow,i_idle,o_idle,i_falling,o_rising")] = Box(5, 0.0, 0.0, 1.0,1.0, 1.0, 2.0, 1.0, 2.0, 4.0,10.0);
    //initial_set[DiscreteLocation("no_overflow,i_idle,o_idle,i_falling,o_falling")] = Box(4, 0.0, 0.0, 0.0,0.0, 1.0, 2, 1.0, 2);
    //initial_set[DiscreteLocation("no_overflow,i_idle,o_idle,i_rising,o_falling")] = Box(4, 1.0, 1.0, 0.0,0.0, 1.0, 2.0, 1.0, 2.0);
    //initial_set[DiscreteLocation("overflow,i_idle,o_idle,i_rising,i_rising")] = Box(4, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0);
    //initial_set[DiscreteLocation("overflow,i_idle,o_idle,i_falling,o_falling")] = Box(4, 0.0, 0.0, 1.0, 2.0, 1.0, 2.0);

    analyse(system, initial_set, verb, plot_results);
    return 0;
}