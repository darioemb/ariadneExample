#pragma once

#include "ariadne.h"

using namespace Ariadne;

/// Forward declarations, used only to properly organize the source file
void finite_time_upper_evolution(HybridAutomatonInterface &system, HybridBoundedConstraintSet &initial_set, int verbosity, bool plot_results);
void finite_time_lower_evolution(HybridAutomatonInterface &system, HybridBoundedConstraintSet &initial_set, int verbosity, bool plot_results);
HybridEvolver::EnclosureListType _finite_time_evolution(HybridAutomatonInterface &system, HybridBoundedConstraintSet &initial_set, Semantics semantics, int verbosity);

// The main method for the analysis of the system
// Since the analyses are independent, you may comment out any one if you want
// to focus on specific ones.
void analyse(HybridAutomatonInterface &system, HybridBoundedConstraintSet &initial_set, int verbosity, bool plot_results)
{
    cout << "1/2: Finite time upper evolution... " << endl
         << flush;
    finite_time_upper_evolution(system, initial_set, verbosity, plot_results);
    cout << "2/2: Finite time lower evolution... " << endl
         << flush;
    finite_time_lower_evolution(system, initial_set, verbosity, plot_results);
}

// Performs finite time evolution.
HybridEvolver::EnclosureListType _finite_time_evolution(HybridAutomatonInterface &system, HybridBoundedConstraintSet &initial_set, Semantics semantics, int verbosity)
{

    // Creates an evolver
    HybridEvolver evolver(system);
    evolver.verbosity = verbosity;
    evolver.settings().set_maximum_step_size(0.1); // The time step size to be used

    // Creates a list of initial enclosures from the initial set.
    // This operation is only necessary since we provided an initial set expressed as a constraint set
    HybridEvolver::EnclosureListType initial_enclosures;
    HybridBoxes initial_set_domain = initial_set.domain();
    for (HybridBoxes::const_iterator it = initial_set_domain.locations_begin(); it != initial_set_domain.locations_end(); ++it)
    {
        if (!it->second.empty())
        {
            initial_enclosures.adjoin(HybridEvolver::EnclosureType(it->first, Box(it->second.centre())));
        }
    }

    // The maximum evolution time, expressed as a continuous time limit along with a maximum number of events
    // The evolution stops for each trajectory as soon as one of the two limits are reached
    HybridTime evol_limits(44.0, 20);

    // Performs the evolution, saving only the reached set of the orbit
    HybridEvolver::EnclosureListType result;
    for (HybridEvolver::EnclosureListType::const_iterator it = initial_enclosures.begin(); it != initial_enclosures.end(); ++it)
    {
        HybridEvolver::OrbitType orbit = evolver.orbit(*it, evol_limits, semantics);
        result.adjoin(orbit.reach());
    }

    return result;
}

// Performs finite time upper evolution
void finite_time_upper_evolution(HybridAutomatonInterface &system, HybridBoundedConstraintSet &initial_set, int verbosity, bool plot_results)
{

    // Performs the evolution, saving only the reached set of the orbit
    HybridEvolver::EnclosureListType reach = _finite_time_evolution(system, initial_set, UPPER_SEMANTICS, verbosity);

    // Plots the reached set specifically
    if (plot_results)
    {
        PlotHelper plotter(system);
        plotter.plot(reach, "upper_reach");
    }
}

// Performs finite time lower evolution.
void finite_time_lower_evolution(HybridAutomatonInterface &system, HybridBoundedConstraintSet &initial_set, int verbosity, bool plot_results)
{

    // Performs the evolution, saving only the reached set of the orbit
    HybridEvolver::EnclosureListType reach = _finite_time_evolution(system, initial_set, LOWER_SEMANTICS, verbosity);

    // Plots the reached set specifically
    if (plot_results)
    {
        PlotHelper plotter(system);
        plotter.plot(reach, "lower_reach");
    }
}

