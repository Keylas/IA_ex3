package ch.epfl.iagents;

import logist.task.Task;
import logist.topology.Topology;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

public class State {

	//The variables that actually make the state: inCity and taskStatuses
	Topology.City inCity;
	Task[] tasks;
	int[] taskStatuses;

	//define constant for readability
	static final int NOT_PICKED = 0, HOLDING = 1, DELIVERED = 2;

	/* These two variables are derived from taskStatuses and kept to avoid recomputing them
	 * weightCarried is the sum of the weights of the tasks currently held
	 * delivered is the number of tasks delivered
	 * A state is final if delivered = #tasks
	 */
	int remainingCapacity = 0;
	int delivered = 0;

	/*
	 * Variables used during the research algorithm.
	 * previousState allows to reconstruct the path after completion
	 * costToReach = previousState.costToReach + previousState.inCity.distanceTo(this.inCity)
	 * heurist is the heuristic for this state, which computation is based on taskStatuses
	 */
	State previousState = null;
	/*
	 * The task that was picked up or delivered from the previous state to arrive at this state.
	 * For the root state it will be null.
	 */
	Task transitionTask = null;

	double costToReach;
	double estimatedCost;

	Heuristic heuristic;

	State(Topology.City cCity, Task[] tasks, int[] tStatus, State previousState,
		  double cost, int remainingCapacity, int delivered, Heuristic heuristic) {
		this.inCity = cCity;
		this.tasks = tasks;
		this.taskStatuses = tStatus;
		this.previousState = previousState;
		this.costToReach = cost;
		this.remainingCapacity = remainingCapacity;
		this.delivered = delivered;
		this.heuristic = heuristic;

		this.estimatedCost = heuristic.getEstimation(this);
	}

	private State(Topology.City cCity, Task[] tasks, int[] tStatus, State previousState,
		  double cost, int remainingCapacity, int delivered, Heuristic heuristic, Task transitionTask) {
		this(cCity, tasks, tStatus, previousState, cost, remainingCapacity, delivered, heuristic);
		this.transitionTask = transitionTask;

		taskStatuses[transitionTask.id] ++; // NOT_PICKED -> HOLDING ; HOLDING -> DELIVERED.
		if (taskStatuses[transitionTask.id] == DELIVERED) this.delivered++;
	}

	private State applyTask(Task t) {
		// Task is not yet delivered.
		// NOT_PICKED -> we will pick up the task.
		// HOLDING -> we will deliver the task.
		Topology.City destination = taskStatuses[t.id] == HOLDING ? t.deliveryCity : t.pickupCity;
		int newRemainingCapacity = remainingCapacity + (taskStatuses[t.id] == HOLDING ? t.weight : -t.weight);
		return new State(
				destination,
				tasks,
				Arrays.copyOf(taskStatuses, taskStatuses.length),
				this,
				costToReach + inCity.distanceTo(destination),
				newRemainingCapacity,
				delivered,
				heuristic,
				t);
	}

	/*
	 * Return a list containing all the possible successors of this state
	 * it is obtained by going through the task list and trying to make progress for each one if possible.
	 */
	List<State> getSuccessors() {
		ArrayList<State> states = new ArrayList<>();
		for (Task t : tasks) {
			if (taskStatuses[t.id] == HOLDING ||
				(taskStatuses[t.id] == NOT_PICKED && t.weight <= remainingCapacity))
				states.add(applyTask(t));
		}
		return states;
//        return tasks.stream()
//                .filter(t -> taskStatuses[t.id] == HOLDING ||
//                        (taskStatuses[t.id] == NOT_PICKED && t.weight <= remainingCapacity))
//                .map(this::applyTask);
	}

	// The distance function: f=g+h=costToReach+heurist
	double f() {
		return this.costToReach + this.estimatedCost;
	}


	//Override hashCode() and equals to be able to use State in HashMap and TreeSet
	@Override
	public int hashCode() {
		return inCity.id + Arrays.hashCode(taskStatuses);
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null) return false;
		if (!(obj instanceof State)) return false;
		if (this == obj)
			return true;
		State other = (State) obj;
		return inCity == other.inCity && Arrays.equals(taskStatuses, other.taskStatuses);
	}

}
