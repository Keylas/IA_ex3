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
	Task[] taskArray;

	/* the properties of the agent */
	Agent agent;
	int capacity;

	/* the planning class */
	Algorithm algorithm;

	State currentState;


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

	private State initState(Vehicle vehicle) {
		int carry=0;
		int[] tStatus = new int[taskArray.length];
		if(currentState!=null) {
			for(int i=0; i<taskArray.length; i++) {	
				tStatus[i]=currentState.taskStatus[i];
				if(tStatus[i]==1) {carry+=taskArray[i].weight;}
			}
		}
		return new State(vehicle.getCurrentCity().id, tStatus, null, 0, carry);
	}

	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;


		this.taskArray = new Task[tasks.size()];
		for(Task t:tasks) {
			taskArray[t.id]=t;
		}

		currentState  = initState(vehicle);

		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			plan = aStarPlan();
			break;
		case BFS:
			// ...
			plan = naivePlan(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}

	private Plan DFSPlan(City current, TaskSet tasks, double best, Plan sketch) {

		Plan plan=new Plan(current);

		return plan;
	}



	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {

		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
		}
	}


	private Plan extractPlan(State finalState) {
		ArrayList<State> l = new ArrayList<State>();
		State s = finalState;
		l.add(s);
		while(s.previousState!=null) {
			l.add(0, s.previousState);
			s=s.previousState;
		}

		Plan p = new Plan(cities[l.get(0).currentCity]);

		for(int i=0; i<l.size()-1; i++) {
			State s1=l.get(i);
			State s2=l.get(i+1);
			if(s1.currentCity!=s2.currentCity) {
				for(City c:cities[s1.currentCity].pathTo(cities[s2.currentCity])) {
					p.appendMove(c);
				}
			}
			for(int j=0; j<taskArray.length; j++) {
				if(s1.taskStatus[j]!=s2.taskStatus[j]) {
					if(s1.taskStatus[j]==0) {
						p.appendPickup(taskArray[j]);
					} else {
						p.appendDelivery(taskArray[j]);
					}
				}
			}
		}

	return p;
}


private Plan aStarPlan() {

	TreeSet<State> q = new TreeSet<State>(new StateComparator()); 
	q.add(currentState);
	HashMap<State,Double> c = new HashMap<State,Double>();
	boolean notFinished = true;
	State node=null;
	while(notFinished) {
		node = q.pollFirst();
		if(node.delivered==taskArray.length) {break;}
		if(!c.containsKey(node) || c.get(node)>(node.cost+node.f())) {
			c.put(node, node.f());
			for(State s: node.successors()) {
				q.add(s);
			}
		}
	}
	return extractPlan(node);

}

private class StateComparator implements Comparator<State>{

	public int compare(State o1, State o2) {
		if(o1.cost+o1.heurist>=o2.cost+o2.heurist) {
			return 1;
		}
		return -1;
	}

}

private class State{

	public State previousState;
	public int currentCity;
	public int[] taskStatus;
	public double cost;
	public double heurist;
	public int delivered;
	public int carry;

	public State(int cCity, int[] tStatus, State previousState, double cost, int carry) {
		this.currentCity=cCity;
		this.taskStatus=tStatus;
		this.previousState=previousState;
		this.cost=cost;
		this.carry=carry;

		this.delivered=0;
		this.heurist=0.0;
		double h=0.0;
		for(int i=0; i<taskStatus.length; i++) {
			switch(taskStatus[i]) {
			case 0:
				h=cities[currentCity].distanceTo(taskArray[i].pickupCity)+cities[taskArray[i].pickupCity.id].distanceTo(taskArray[i].deliveryCity);
				break;
			case 1:
				h=cities[currentCity].distanceTo(taskArray[i].deliveryCity);
				break;
			case 2:
				this.delivered++;
				break;
			}
			if(h>heurist) {heurist=h;}
		}
	}


	public ArrayList<State> successors(){
		ArrayList<State> succ = new ArrayList<State>();
		for(int i=0; i<taskStatus.length; i++) {
			if(taskStatus[i]==1 || (taskStatus[i]==0 && taskArray[i].weight+this.carry<=capacity)) {
				int[] tmp = Arrays.copyOf(this.taskStatus, this.taskStatus.length);
				tmp[i]++;
				int nextCity, nextCarry;
				if(taskStatus[i]==0) {nextCity=taskArray[i].pickupCity.id; nextCarry=this.carry+taskArray[i].weight;}
				else {nextCity=taskArray[i].deliveryCity.id; nextCarry=this.carry-taskArray[i].weight;}
				succ.add(new State(nextCity, tmp, this, cities[currentCity].distanceTo(cities[nextCity]), nextCarry));
			}
		}
		return succ;
	}


	public int hashCode() {
		return currentCity+Arrays.hashCode(taskStatus);
	}

	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		State other = (State) obj;
		if (currentCity != other.currentCity)
			return false;
		if (!Arrays.equals(taskStatus, other.taskStatus))
			return false;
		return true;
	}



	public double f() {
		return this.cost+this.heurist;
	}



}

}
