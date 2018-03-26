#include <ariadne.h>
#include "dam.hh"
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
    initial_set[DiscreteLocation("flow,a_idle,b_idle,p_falling,t_falling")] = Box(5, 0.0,0.0, 0.01,0.01, 1.0,100.0, 1.0,10.0, 25.0,30.0);

    analyse(system, initial_set, verb, plot_results);
    return 0;
}
