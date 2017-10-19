package ch.epfl.iagents;

import java.util.LinkedList;
import java.util.stream.Collectors;

public class BFSPlanner extends Planner {

	@Override
	protected State getFinalState(State initialState) {
		final int deliverableTaskCount = initialState.getDeliverableTaskCount();
		LinkedList<State> q = new LinkedList<>();
		q.add(initialState);

		State bestState = null;
		Double bestResult = Double.POSITIVE_INFINITY;
		while (!q.isEmpty()) {
			State s = q.poll();
			if (s.delivered == deliverableTaskCount && s.costToReach < bestResult) {
				bestResult = s.costToReach;
				bestState = s;
			} else {
				//s.getSuccessors().collect(Collectors.toCollection(() -> q));
				q.addAll(s.getSuccessors());
			}
		}

		return bestState;
	}
}
