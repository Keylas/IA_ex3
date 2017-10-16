package template;

/* import table */
import logist.simulation.Vehicle;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.TreeMap;
import java.util.TreeSet;

import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeMain implements DeliberativeBehavior {

	enum Algorithm { BFS, ASTAR }

	/* Environment */
	Topology topology;
	City[] cities;
	HashMap<Integer,Task> taskMap;
	TaskSet carriedTasks;

	/* the properties of the agent */
	Agent agent;
	int capacity;

	/* the planning class */
	Algorithm algorithm;

	State initialState;


	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		this.topology = topology;
		this.agent = agent;

		this.cities = new City[topology.size()];
		for(City city:topology) {
			cities[city.id]=city;
		}

		// initialize the planner
		this.capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		// Throws IllegalArgumentException if algorithm is unknown
		algorithm = Algorithm.valueOf(algorithmName.toUpperCase());

		// ...
	}

	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;

		initialize(vehicle, tasks); //take care of setting taskMap and initialState correctly

		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			plan = aStarPlan();
			break;
		case BFS:
			// ...
			plan = aStarPlan();
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}


	@Override
	public void planCancelled(TaskSet carriedTasks) {

		if (!carriedTasks.isEmpty()) {
			/*register the carriedTask
			 *they will  be taken into account by initialize when plan() is called
			 */
			this.carriedTasks=carriedTasks;
		}
	}

	private void initialize(Vehicle vehicle, TaskSet tasks) {

		
		
		int[] taskStatus;
		int carry = 0;

		this.taskMap = new HashMap<Integer,Task>();
		int i=0;
		if(carriedTasks!=null) {
			taskStatus = new int[carriedTasks.size()+tasks.size()];
			for(Task t:carriedTasks) {
				taskMap.put(i, t);
				taskStatus[i]=1;
				carry+=t.weight;
				i++;
			}
		} else {taskStatus = new int[tasks.size()];}
		for(Task t:tasks) {
				taskMap.put(i, t);
				i++;
		}
		
		this.initialState = new State(vehicle.getCurrentCity(), taskStatus, null, 0.0, carry, true);
		
	}


	private Plan DFSPlan(City current, TaskSet tasks, double best, Plan sketch) {

		Plan plan=new Plan(current);

		return plan;
	}


	//Implementation of ASTAR algorithm
	private Plan aStarPlan() {

		//Set of "border" nodes. The use of TreeSet automatically sort them on insertion
		TreeSet<State> q = new TreeSet<State>(new StateComparator());
		q.add(initialState); //start with origin

		/*Set of node "seen". Because our heuristic is not consistent, we'll have to compare the cost of an already-seen node
		 * if we reach it again, hence the use of HashMap<State,f(State)>
		 */
		HashMap<State,Double> c = new HashMap<State,Double>();


		State node=null; //avoid re-instantiation and used as final node when the loop is over
		while(true) {
			node = q.pollFirst();

			if(node==null || node.delivered==taskMap.size()) {break;} //if node is final

			//if not has not been reached yet, or we reached it with a better cost
			if(!c.containsKey(node) || c.get(node)>node.f()) {
				c.put(node, node.f());

				for(State s: node.successors(true)) {
					q.add(s);
				}
			}
		}
		//we found our solution with node being the final node, extract the corresponding plan and return it
		if(node==null) {return Plan.EMPTY;}
		return extractPlan(node);

	}

	private Plan extractPlan(State finalState) {
		ArrayList<State> l = new ArrayList<State>();
		State s = finalState;
		l.add(s);
		while(s.previousState!=null) {
			l.add(0, s.previousState);
			s=s.previousState;
		}

		Plan p = new Plan(l.get(0).inCity);

		for(int i=0; i<l.size()-1; i++) {
			State s1=l.get(i);
			State s2=l.get(i+1);
			if(s1.inCity!=s2.inCity) {
				for(City c:s1.inCity.pathTo(s2.inCity)) {
					p.appendMove(c);
				}
			}
			for(int j=0; j<taskMap.size(); j++) {
				if(s1.taskStatus[j]!=s2.taskStatus[j]) {
					if(s1.taskStatus[j]==0) {
						p.appendPickup(taskMap.get(j));
					} else {
						p.appendDelivery(taskMap.get(j));
					}
				}
			}
		}

		System.out.println("Distance of plan: "+p.totalDistance());
		return p;
	}



	private class State{

		//The variables that actually make the state: inCity and taskStatus
		public City inCity;
		public int[] taskStatus;

		//define constant for readability
		private static final int NOT_PICKED=0, HOLDING=1, DELIVERED=2;

		/* These two variables are derived from taskStatus and kept to avoid recomputing them
		 * weightCarried is the sum of the weights of the tasks currently held
		 * delivered is the number of tasks delivered
		 * A state is final if delivered = #tasks
		 */
		public int weightCarried=0;
		public int delivered=0;

		/*
		 * Variables used during the research algorithm.
		 * previousState allows to reconstruct the path after completion
		 * costToReach = previousState.costToReach + previousState.inCity.distanceTo(this.inCity)
		 * heurist is the heuristic for this state, which computation is based on taskStatus
		 */
		public State previousState;

		public double costToReach;
		public double heurist=0.0;


		public State(City cCity, int[] tStatus, State previousState, double cost, int carry, boolean computeHeurist) {
			this.inCity=cCity;
			this.taskStatus=tStatus;
			this.previousState=previousState;
			this.costToReach=cost;
			this.weightCarried=carry;

			if(computeHeurist) {this.computeHeurist();} //comupute heuristic if necessary (we're using Astar)
		}

		/*
		 * Compute the heuristic for this state (called during instantiation)
		 * Since the heuristic must be admissible (h(s)<=d(s), the true distance),
		 * we chose h(s)=max{task t not delivered}{ inCity.distanceTo(pickup(t))+pickup(t).distanceTo(delivery(t)) } [in t is not picked up yet]
		 * 										  {	inCity.distanceTo(delivery(t)) } [if t is currently being held]
		 */
		private void computeHeurist() {
			double h=0.0; //declared here to avoid redeclaration

			//find a maximum on all tasks
			for(int i=0; i<taskStatus.length; i++) {
				switch(taskStatus[i]) {
				case NOT_PICKED:
					//Not picked=> we will have to go from where we are to the pickup city, then to the delivery city
					h=inCity.distanceTo(taskMap.get(i).pickupCity)+cities[taskMap.get(i).pickupCity.id].distanceTo(taskMap.get(i).deliveryCity);
					break;
				case HOLDING:
					//Holding=> we will have to go from where we are to the delivery city
					h=inCity.distanceTo(taskMap.get(i).deliveryCity);
					break;
				case DELIVERED:
					//Delivered=>Nothing to do, mark that we delivered.
					this.delivered++;
					break;
				}
				if(h>this.heurist) {this.heurist=h;}
			}
		}

		//The distance function: f=g+h=costToReach+heurist
		public double f() {
			return this.costToReach+this.heurist;
		}


		/*return a list containing all the possible successors of this state
		 * it is obtained by going through the task list and trying to make progress for each one if possible
		 */
		public ArrayList<State> successors(boolean computeHeurist){ //computeHeurist=true if we're using Astar

			ArrayList<State> successorList = new ArrayList<State>();

			for(int i=0; i<taskStatus.length; i++) { //For each task, we can make it progress...
				//if we are holding it (our move will be to deposit it), or if we have the capacity to pick it up (our move will be to pick it up)
				if(taskStatus[i]==HOLDING || (taskStatus[i]==NOT_PICKED && taskMap.get(i).weight+this.weightCarried<=capacity)) {

					int[] tmp = Arrays.copyOf(this.taskStatus, this.taskStatus.length); //make a true copy of the current state
					tmp[i]++; //register our move by updating the corresponding task

					int nextWeightCarry; City nextCity;
					//lookup what is the inCity for this next State and update the weight carried if necessary
					if(taskStatus[i]==NOT_PICKED) {nextCity=taskMap.get(i).pickupCity; nextWeightCarry=this.weightCarried+taskMap.get(i).weight;}
					else {nextCity=taskMap.get(i).deliveryCity; nextWeightCarry=this.weightCarried-taskMap.get(i).weight;}

					successorList.add(new State(nextCity, tmp, this, inCity.distanceTo(nextCity), nextWeightCarry, computeHeurist));
				}
			}
			return successorList;
		}


		//Override hashCode() and equals to be able to use State in HashMap and TreeSet
		@Override
		public int hashCode() {
			return inCity.id+Arrays.hashCode(taskStatus);
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			State other = (State) obj;
			if (inCity != other.inCity)
				return false;
			if (!Arrays.equals(taskStatus, other.taskStatus))
				return false;
			return true;
		}

	}

	//StateComparator required for TreeMap<State>
	private class StateComparator implements Comparator<State>{
		public int compare(State o1, State o2) {
			if(o1.costToReach+o1.heurist>=o2.costToReach+o2.heurist) {
				return 1;
			}
			return -1;
		}
	}

}
