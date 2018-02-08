package ch.epfl.iagents;

import java.util.*;
import java.util.stream.Collectors;

public class AStarPlanner extends Planner {
	@Override
	protected State getFinalState(State initialState) {
		final int deliverableTaskCount = initialState.getDeliverableTaskCount();
		// Set of "border" nodes. The use of TreeSet automatically sort them on insertion
		TreeSet<State> q = new TreeSet<>(new StateComparator());
		q.add(initialState); //start with origin

		/*
		 * Set of node "seen". Because our heuristic is not consistent, we'll have to compare the cost of an already-seen node
		 * if we reach it again, hence the use of HashMap<State,f(State)>
		 */
		HashMap<State, Double> c = new HashMap<>();

		int numLoop = 0; //feedback info
		State s; //avoid re-instantiation and used as final node when the loop is over
		do {
			numLoop++;

			s = q.pollFirst();
			if (s.delivered == deliverableTaskCount) break;
			//if not has not been reached yet, or we reached it with a better cost
			if (!c.containsKey(s) || c.get(s) > s.f()) {
				c.put(s, s.f());
				//s.getSuccessors().collect(Collectors.toCollection(() -> q));
				q.addAll(s.getSuccessors());
			}
			// System.out.println(q.size());
		} while (!q.isEmpty());
		System.out.println(c.size() + " estimated, " + q.size() + " border, " + numLoop + " loops, final cost: "
				+ s.costToReach +" using ASTAR.");
		return s;
	}

	// StateComparator required for TreeMap<State>
	private class StateComparator implements Comparator<State> {
		public int compare(State o1, State o2) {
			int r = Double.compare(o1.f(), o2.f());
			if (r != 0) return r;
			// Hackish: if the states are equidistant, break the tie.
			return o1.equals(o2) ? 0 : 1;
		}
	}
}
