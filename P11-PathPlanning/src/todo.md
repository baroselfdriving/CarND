# TODO

- deal with vehicle barging in to our path on short notice
- avoiding collisions when changing lane
- comment out all std::cout

# Behaviours implementation

- Prediction: For all vehicles closer than threshold distance, create predicted trajectories
- cost functions: collisions, safe distance, deviation from target speed, lane change cost

# Report points

- Tragen polynomial in frenet coords. Generate for velocity rather than position.
- Numerical integration.
- Graph showing trajectory for unobstructed lane change
- Problem with generating trajectories in frenet coords for bendy roads where acc constraints can be crossed for straight frenet segments
- Modified spline lib
- vehicle model used and control inputs
- Error formulation and PID control for speed and steering - mainly for containing lateral accelerations. Gains are
  tuned for gentle control changes.
- prediction logic - constant velocity constant lane
- behaviours cost functions and weights
- Brief description of classes and functionality
