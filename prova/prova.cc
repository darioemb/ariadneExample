#include<ariadne.h>
#include"system.h"
#include"analysis.h"


int main(int argc, char**argv)
{
	RealParameter T("T",4.0);

	HybridIOAutomaton tick("tick");
	DiscreteLocation timer("timer");
	DiscreteEvent clock("clock");

	RealVariable x("x");

	tick.add_internal_var(x);

	RealExpression x_d=1;

	tick.new_mode(timer);
	tick.set_dynamics(timer, x, x_d);

	RealExpression guard = x-T;
	std::map<RealVariable, RealExpression> reset;
	reset[x]=0;

	tick.new_forced_transition(clock, timer, timer, reset, guard);
	cout<< "Automaton = "<<tick<<"\n";

	HybridEvolver evolver(tick);

	evolver.settings().set_reference_enclosure_widths(0.05);
	evolver.settings().set_maximum_step_size(1.0/64);
	evolver.verbosity=1;
	std::cout<<evolver.settings() <<"\n";

	typedef HybridEvolver::EnclosureType HybridEnclosureType;
	typedef HybridEvolver::OrbitType OrbitType;

	Box initial_box(1,0.0, 4.0);
	HybridEnclosureType initial_enclosure(timer, initial_box);
	Box bounding_box(2, 0.0, 4.0, 0.0, 4.0);

	HybridTime evolution_time(1.0, 4);

	std::cout<<"Computing orbit..."<<std::flush;
	OrbitType orbit=evolver.orbit(initial_enclosure, evolution_time, UPPER_SEMANTICS);
	std::cout<<"done.\n";
	plot("tick", bounding_box, Colour(0.0,0.5,1.0), orbit);

}
