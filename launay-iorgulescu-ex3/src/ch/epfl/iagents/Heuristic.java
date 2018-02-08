package ch.epfl.iagents;

public interface Heuristic {
	double getEstimation(State currentState);
}
