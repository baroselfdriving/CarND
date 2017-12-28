# TODO

- behaviours
- deal with vehicle barging in to our path on short notice
- safe lane following - speed derating
- PID tuning to stay within acceleration, jerk and speed limit
- consider reducing speed for large heading changes

# Behaviours implementation

- Prediction: For all vehicles closer than threshold distance, create predicted trajectories
- cost functions: collisions, safe distance, deviation from target speed, lane change cost
- Behaviours: implement the transition function pseudocode

def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights)
{
  availableSuccessorStates = getSuccessorStates(current_fsm_state);

  for(availableState : availableSuccessorStates)
  {
    trajectoryForState = generateTrajectory(availableState, currentPose, predictions);
    cost = 0;
    for( costFunc : cost_functions)
    {
      cost += weights[costFuncIndex] * costFunc(trajectoryForState, predictions);
    }
    costMap[availableState] = cost;
  }

  nextState = getStateWithMinCost(costMap, availableStates);
  return nextState;
}

# Report points

- Tragen polynomial in frenet coords. Generate for velocity rather than position.
- Numerical integration.
- Graph showing trajectory for unobstructed lane change
- Problem with generating trajectories in frenet coords for bendy roads where acc constraints can be crossed for straight frenet segments
- Modified spline lib
- vehicle model used and control inputs
- Error formulation and PID control for speed and steering - mainly for containing lateral accelerations. Gains are
  tuned for gentle control changes.
- Brief description of classes and functionality
