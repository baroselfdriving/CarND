# Project 11: Path Planning
Self-Driving Car Engineer Nanodegree Program (Term 3)
---
## Executive Summary
The submitted source code implements an autonomous path planner for a simulated car on a 3-lane highway. As shown in the image below, the car is able to continuously drive without incident multiple loops around the track. Incidents to consider included collisions, departing lanes inadvertently, exceeding acceleration and jerk limits (10m/s^2 and 10 m/s^3, respectively), and exceeding speed limit of 50 miles per hour.

![autonomous path planning](https://github.com/cvilas/CarND/blob/master/P11-PathPlanning/report/media/t3p1-noIncident.png "Driving without incident for multiple loops")


https://youtu.be/-0TvdNeugxE

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
