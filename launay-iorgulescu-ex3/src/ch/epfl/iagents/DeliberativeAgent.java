package ch.epfl.iagents;

/* import table */

import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
import logist.simulation.Vehicle;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;

import java.util.Arrays;

import static ch.epfl.iagents.State.HOLDING;
import static ch.epfl.iagents.State.NOT_PICKED;
import static ch.epfl.iagents.State.UNAVAILABLE;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeAgent implements DeliberativeBehavior {

	/*
	 * Maximum (few seconds responses):
	 * BFS => 5 tasks
	 * ASTAR:NONE => 10 tasks
	 * ASTAR:MAXTRIP => 11 tasks
	 */

	/* Environment */
	boolean environmentInitialized = false;
	TaskSet carriedTasks;
	Task[] tasks;

	/* the properties of the agent */
	int capacity;

	Planner planner;
	Heuristic heuristic;

	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		// initialize the planner
		this.capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		// Throws IllegalArgumentException if algorithm is unknown
		planner = Planner.getPlanner(Algorithm.valueOf(algorithmName.toUpperCase()));

		String heuristicName = agent.readProperty("heuristic", String.class, "MAXTRIP");
		// Throws IllegalArgumentException if algorithm is unknown
		heuristic = Heuristics.getHeuristic(HeuristicKind.valueOf(heuristicName.toUpperCase()));
	}

	private int[] getTaskStatuses(TaskSet tasks) {
		int[] taskStatuses = new int[this.tasks.length];
		Arrays.fill(taskStatuses, UNAVAILABLE);
		tasks.forEach(t -> taskStatuses[t.id] = NOT_PICKED);
		carriedTasks.forEach(t -> taskStatuses[t.id] = HOLDING);
		return taskStatuses;
	}

	private void initialize(TaskSet tasks) {
		// First call to plan().
		// Setup carriedTasks to have the same universe as the tasks.
		carriedTasks = TaskSet.noneOf(tasks);
		// Setup tasks like taskSet universe.
		// Since some tasks may be done, allocate space for the largest task id found.
		this.tasks = new Task[tasks.stream().map(t -> t.id).mapToInt(Integer::intValue).max().orElse(-1) + 1];
		tasks.forEach(t -> this.tasks[t.id] = t);
		environmentInitialized = true;
	}

	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		if (!environmentInitialized) initialize(tasks);
		if (tasks.isEmpty() && carriedTasks.isEmpty()) return Plan.EMPTY;
		return planner.getPlan(new State(vehicle.getCurrentCity(), this.tasks, getTaskStatuses(tasks),
				tasks.size() + carriedTasks.size(), null, 0.0,
				capacity - carriedTasks.weightSum(), 0, heuristic));
	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {
		/*
		 * Register the carriedTasks.
		 * They will be taken into account when plan() is called
		 */
		this.carriedTasks = carriedTasks;
	}

}
