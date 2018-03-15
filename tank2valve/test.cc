#include <ariadne.h> // Library header
#include "tank2valve.hh" // System definition
#include "analysis.hh" // Custom analysis routines to be run

int main(int argc,char *argv[])
{

    int verb = 0;
    if (argc > 1)
        verb = atoi(argv[1]);

    bool plot_results = true;

    HybridIOAutomaton system = Ariadne::getSystem();

    cout << system << endl;

    HybridBoundedConstraintSet initial_set(system.state_space());
    initial_set[DiscreteLocation("flow,i_idle,o_idle,i_rising,o_rising")] = Box(3, 1.0,1.0, 0.0,0.0, 6.0,7.5);
    initial_set[DiscreteLocation("flow,i_idle,o_idle,i_falling,o_falling")] = Box(3, 0.0,0.0, 1.0,1.0, 6.0,7.5);

    analyse(system,initial_set,verb,plot_results);
}

