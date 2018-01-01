# TODO

- exceeds acceleration limit at track boundary for one cycle
- exceeds acceleration limit when starting from stop/low speed
- separate out response_time from ramp_up_time
- vehicle starts from zero turned backwards
- deal with vehicle barging in to our path on short notice. Clear history and path and start again
- avoiding collisions when changing lane

# Report points

- Tragen polynomial in frenet coords. Generate for velocity rather than position.
- Numerical integration.
- Graph showing trajectory for unobstructed lane change
- Problem with generating trajectories in frenet coords for bendy roads where acc constraints can be crossed for straight frenet segments
- Modified spline lib
- vehicle model used and control inputs
- Error formulation and PID control for speed and steering - mainly for containing lateral accelerations. Gains are
  tuned for gentle control changes.
- prediction logic - constant velocity constant lane. highly conservative, but still not perfect. Low speed
  collisions are possible
- using min_response_time as the criterion for speed control and lane changes (based on safe driving heuristics)
- behaviours cost functions and weights
- Brief description of classes and functionality

- Limitations
  - Requires 3 seconds response time. If vehicle barges into path with less than 3 sec to collision,
  collision cannot be averted.
  - Only considers changing into the next nearest lane
  - Conservative unspohisticated motion model for predictions
  - conservative lane change strategy. First priorty is to stay safe within the current lane
