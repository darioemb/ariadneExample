#include <ariadne.h>
#include "lock_system.hh"
#include "analysis.hh" 
int main(int argc, char **argv)
{
    int verb = 0;
    if (argc > 1)
        verb = atoi(argv[1]);

    bool plot_results = true;

    HybridIOAutomaton system = Ariadne::getSystem();

    cout << system << endl;

    HybridBoundedConstraintSet initial_set(system.state_space());
    initial_set[DiscreteLocation("passon,opening")] = Box(4, 0.0,0.0, 0.0,0.0, 0.0,0.0, 1.0,1.0 );


    analyse(system, initial_set, verb, plot_results);
    return 0;
}
