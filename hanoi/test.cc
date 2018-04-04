#include <ariadne.h>
#include "hanoi.hh"
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

    //initial_set[DiscreteLocation("flow,opening12,idle13,idle23,idle21,idle31,idle32,S0")] = Box(9, 0.0,0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0, 0.0,8.0, 0.0,0.0, 0.0,0.0);


    analyse(system, initial_set, verb, plot_results);
    return 0;
}
