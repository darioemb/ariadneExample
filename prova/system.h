/***************************************************************************
 *            system.h
 *
 *  This file provides the system definition.
 *  Specifically, this is watertank system in which a tank with a hole in the
 *  bottom receives an input water flow. Such input flow can
 *  be modulated between zero and its maximum by controlling a valve. The
 *  described controller aims at keeping the water level between an upper threshold
 *  and a lower threshold.
 *
 *  Copyright  2017  Luca Geretti
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#pragma once

#include <ariadne.h>

namespace Ariadne {

/*
 * Construction of an automaton takes major six steps:
 *
 * 1. Creation of an automaton
 * 2. Registration of variable on the automaton
 * 3. Registration of events on the automaton
 * 4. Registration of locations as modes of the automaton
 * 5. Registration of dynamics for each mode
 * 6. Registration of transitions from one mode to another mode
 *
 * As a 0-th step, we also need to create (valued) labels: system variables, parameters and events
 *
 * The creation of labels was numbered 0 since they may be shared between multiple automata, and we clearly
 * need to define them only once in the given context, before any usage.
 *
 * While it may be convenient to define all the components of a system on the same context,
 * for sufficiently complex automata it becomes preferable to separate automata within
 * dedicated (header) files. In that case, shared labels must be redefined within each context.
 *
 * Finally, a system is obtained by automated composition of its components.
 */

HybridIOAutomaton getSystem()
{
    // 0: System variables

		RealVariable s("s"); // tick
		RealParameter T("T", 1.0);

    /// Tick automaton


		// 1. Automaton

			HybridIOAutomaton tick("tick");

		// 2. Registration of the input/output variables

			tick.add_output_var(s);

		// 3. Registration of input/output events
			
			DiscreteEvent e_clock("clock");

			tick.add_output_event(e_clock);

		// 4. Registration of the locations

			DiscreteLocation timer("timer");

			tick.new_mode(timer);

		/// 5. Registration of the dynamics

			tick.set_dynamics(timer, s, 1);

		/// 6. Transitions
			
			// The library assumes that given a guard g, the relation g >= 0 must hold in the current mode to have a transition
			RealExpression s_geq_T = s-T; // s >= T

			// Resets
			// We need to define a reset for each output variable of the automaton
			std::map<RealVariable,RealExpression> rst_s_zero;
			rst_s_zero[s] = 0.0; // a = 0

			tick.new_forced_transition(e_clock, timer, timer, rst_s_zero, s_geq_T);


    return tick;
}


}

