#include <ariadne.h>
#include "hydropower.hh"
#include "analysis.hh" // Custom analysis routines to be run

int main(int argc, char **argv)
{
    int verb = 0;
    if (argc > 1)
        verb = atoi(argv[1]);

    bool plot_results = true;

    HybridIOAutomaton system = Ariadne::getSystem(9.81,0.85,10.0,100.0,0.01,0.1,0.2,0.01,4.0,100.0,300.0);

    cout << system << endl;
    
    HybridBoundedConstraintSet initial_set(system.state_space());    
    //with time fixed to first position
    initial_set[DiscreteLocation("flow,a_idle,p_falling,day")] = Box(5, 0.0,0.0, 0.0,0.0, 0.0,0.0, 1.0,100.0, 1.0,900.0);

    analyse(system, initial_set, verb, plot_results);
    return 0;
}
