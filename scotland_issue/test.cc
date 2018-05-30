#include <ariadne.h>
#include "scotland_issue.hh"
#include "analysis.hh" // Custom analysis routines to be run

int main(int argc, char **argv)
{
    int verb = 0;
    if (argc > 1)
        verb = atoi(argv[1]);

    bool plot_results = true;

    HybridIOAutomaton system = Ariadne::getSystem();

    // cout << system << endl;

    HybridBoundedConstraintSet initial_set(system.state_space());
    initial_set[DiscreteLocation("no_overflow,tick,opening")] = Box(5, 0.0,0.0, 0.0,0.0, 3.0,3.0, 0.1,0.1, 1.0,1.0);

    analyse(system, initial_set, verb, plot_results);
    return 0;
}
