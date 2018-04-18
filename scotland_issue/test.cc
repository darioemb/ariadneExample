#include <ariadne.h>
#include "scotland_issue.hh"
#include "analysis.hh" // Custom analysis routines to be run

int main(int argc, char **argv)
{
    int verb = 0;
    if (argc > 1)
        verb = atoi(argv[1]);

    bool plot_results = true;

    HybridIOAutomaton system = Ariadne::getSystem(0.01,0.06,0.02,0.01,0.01,2.0,1.0,2.0,0.01,1.9,1.9);

    cout << system << endl;

    HybridBoundedConstraintSet initial_set(system.state_space());
    initial_set[DiscreteLocation("S0,idle,tick,rising")] = Box(6, 0.0,0.0, 1.0,1.0, 0.0,0.0, 1.0,2.0, 1.0, 2.0, 1.0, 2.0);


    //initial_set[DiscreteLocation("S0,idle,falling")] = Box(5, 0.0,0.0, 0.0,0.0, 1.0,2.0, 1.0, 2.0, 1.0, 2.0);

    analyse(system, initial_set, verb, plot_results);
    return 0;
}
