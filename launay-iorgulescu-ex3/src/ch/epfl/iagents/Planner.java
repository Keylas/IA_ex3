package ch.epfl.iagents;

import logist.plan.Action;
import logist.plan.Plan;
import logist.task.Task;
import logist.topology.Topology;

import java.util.ArrayList;
import java.util.Collections;
import java.util.stream.Collectors;

public abstract class Planner {
	abstract protected State getFinalState(State initialState);

	static Planner getPlanner(Algorithm algorithm)
			throws IllegalArgumentException {
		switch (algorithm) {
			case BFS:
				return new BFSPlanner();
			case ASTAR:
				return new AStarPlanner();
			default:
				throw new IllegalArgumentException("Undefined algorithm type: " + algorithm);
		}
	}

	Plan getPlan(State initialState) {
		assert(initialState != null);

		long startTime = System.currentTimeMillis();
		State state = getFinalState(initialState);
		if (state != null) {
			System.out.println(state.tasks.length + " tasks, result: " + state.costToReach + " in " +
					(System.currentTimeMillis() - startTime) + " ms");
		}

		ArrayList<Action> plan = new ArrayList<>();
		while (state.previousState != null) {
			Task transitionTask = state.transitionTask;

			// Check if the task was delivered or picked up.
			if (state.taskStatuses[transitionTask.id] == State.DELIVERED)
			   plan.add(new Action.Delivery(transitionTask));
			else plan.add(new Action.Pickup(transitionTask));

			// Check if we moved to a different location.
			// We construct the path from destination to source, since we need to reverse it at the end.
			if (state.inCity != state.previousState.inCity) {
				final Topology.City sourceCity = state.previousState.inCity;
				plan.add(new Action.Move(state.inCity));
				plan.addAll(state.inCity.pathTo(state.previousState.inCity)
						.stream().filter(c -> c != sourceCity)
						.map(Action.Move::new).collect(Collectors.toList()));
			}

			state = state.previousState;
		}

		Collections.reverse(plan);
		return new Plan(state.inCity, plan);
	}
}
