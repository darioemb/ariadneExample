#include<ariadne.h>
#include"crazy_river.hh"

#include<iostream>


int main(int argc, char** argv)
{
	HybridIOAutomaton crazy_river=getSystem();

	std::cout<<"Automaton: "<<crazy_river<<"\n";

	HybridBoundedConstraintSet initial_set(crazy_river.state_space());

	initial_set[DiscreteLocation("no_overflow")]=Box(2, 0.0f, 0.0f, 6.0f, 7.5f);

	HybridEvolver evolver(crazy_river);
	evolver.settings().set_maximum_step_size(0.3);

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

	HybridTime evol_limits(30.0, 10);

	HybridEvolver::EnclosureListType reach;
	for(HybridEvolver::EnclosureListType::const_iterator it=initial_enclosures.begin();it!=initial_enclosures.end();++it)
	{
		HybridEvolver::OrbitType orbit=evolver.orbit(*it, evol_limits, UPPER_SEMANTICS);
		reach.adjoin(orbit.reach());
	}

	PlotHelper plotter(crazy_river);
//	plotter.plot(reach, "crazy_river");

	return 0;
}
