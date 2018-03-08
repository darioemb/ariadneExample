#include<ariadne.h>
#include"tick.hh"

#include<iostream>


int main(int argc, char** argv)
{
	HybridIOAutomaton tick=getSystem();

	std::cout<<"Automaton: "<<tick<<"\n";

	HybridBoundedConstraintSet initial_set(tick.state_space());
	initial_set[DiscreteLocation("timer")]=Box(1,0.0, 1.0);

	HybridEvolver evolver(tick);
	evolver.settings().set_maximum_step_size(1);

	evolver.verbosity=1;
	HybridEvolver::EnclosureListType initial_enclosures;
	HybridBoxes initial_set_domain = initial_set.domain();
	for(HybridBoxes::const_iterator it=initial_set_domain.locations_begin();it!=initial_set_domain.locations_end();++it)
	{
		if(!it->second.empty())
			initial_enclosures.adjoin(HybridEvolver::EnclosureType(it->first, Box(it->second.centre())));
	}

	std::cout<<"initial enclosures: "<<initial_enclosures<<"\n";
	std::cout<<"initial set_domain: "<<initial_set_domain<<"\n";
	std::cout<<"initial set: "<<initial_set<<"\n";

	HybridTime evol_limits(40.0, 10);

	HybridEvolver::EnclosureListType reach;
	for(HybridEvolver::EnclosureListType::const_iterator it=initial_enclosures.begin();it!=initial_enclosures.end();++it)
	{
		HybridEvolver::OrbitType orbit=evolver.orbit(*it, evol_limits, UPPER_SEMANTICS);
		reach.adjoin(orbit.reach());
	}

	PlotHelper plotter(tick);
	plotter.plot(reach, "tick");
}
