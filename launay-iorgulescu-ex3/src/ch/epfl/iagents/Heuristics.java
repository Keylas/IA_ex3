package ch.epfl.iagents;

import logist.task.Task;

import java.util.stream.Stream;

class Heuristics {

	static Heuristic getHeuristic(HeuristicKind heuristic) throws IllegalArgumentException {
		switch(heuristic) {
			case MAXTRIP:
				return new MaxTripHeuristic();
			default:
				throw new IllegalArgumentException("Undefined heuristic type: " + heuristic);
		}
	}

	private static class MaxTripHeuristic implements Heuristic {
		@Override
		public double getEstimation(State currentState) {
			/*
			double pickupMax = Stream.of(currentState.tasks)
					.filter(t -> currentState.taskStatuses[t.id] == State.NOT_PICKED)
					.map(t -> currentState.inCity.distanceTo(t.pickupCity) + t.pickupCity.distanceTo(t.deliveryCity))
					.mapToDouble(Double::doubleValue)
					.max().orElse(0);

			double deliverMax = Stream.of(currentState.tasks)
					.filter(t -> currentState.taskStatuses[t.id] == State.HOLDING)
					.map(t -> currentState.inCity.distanceTo(t.deliveryCity))
					.mapToDouble(Double::doubleValue)
					.max().orElse(0);

			return pickupMax > deliverMax ? pickupMax : deliverMax;
			*/
			double result = 0;
			for (Task t : currentState.tasks) {
				double value = 0;
				if (currentState.taskStatuses[t.id] == State.NOT_PICKED) {
					value = currentState.inCity.distanceTo(t.pickupCity) + t.pickupCity.distanceTo(t.deliveryCity);
				} else if (currentState.taskStatuses[t.id] == State.HOLDING) {
					value = currentState.inCity.distanceTo(t.deliveryCity);
				}
				if (value > result) result = value;
			}

			return result;
		}
	}
}
