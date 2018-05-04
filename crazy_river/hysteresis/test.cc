#include <ariadne.h>
#include "crazy_river.hh"
#include "analysis.hh" // Custom analysis routines to be run

int main(int argc, char **argv)
{
    int verb = 0;
    if (argc > 1)
        verb = atoi(argv[1]);

    bool plot_results = true;

    HybridIOAutomaton system = Ariadne::getSystem();

    cout << system << endl;

    HybridBoundedConstraintSet initial_set(system.state_space());
    // initial_set[DiscreteLocation("S0,idle,rising")] = Box(5, 0.0,0.0, 1.0,1.0, 1.0,2.0, 1.0, 2.0, 1.0, 2.0);
    initial_set[DiscreteLocation("no_overflow,idle,falling")] = Box(5, 0.0,0.0, 0.0,0.0, 1.5,2.0, 1.5,2.0, 1.0, 2.0);

    analyse(system, initial_set, verb, plot_results);
    return 0;
}