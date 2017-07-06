# Project 9: PID Control
Self-Driving Car Engineer Nanodegree Program

---
## Introduction
This project implements a PID controller to manouvre a vehicle in the simulated race track. The simulator provided the
cross track error (CTE) and the velocity (mph), and the PID controller is meant to generate the appropriate steering
angle to keep the vehicle within the edges of the track. The next section describes how I achieved controller tuning.

## Discussion
### Describe the effect each of the P, I, D components had in your implementation.

### Describe how the final hyperparameters were chosen.

A basic PID controller was first implemented using the code framework provided. I hand-tuned the control gains to
achieve stable vehicle control in the simulator. There was some swaying, but the vehicle remained within the boundaries
of the track. The trottle was set to preset default of 0.3. The hand-tuned PID gains were [.15, 0.001, 1] respectively.
[Here's the video](output/pid_init.mp4) of the simulated vehicle running with these PID parameters.

## Build Instructions

- Clone this repo.
- Install websockets library: `sh ./install-ubuntu.sh`
- Make a build directory: `mkdir build && cd build`
- Compile: `cmake .. && make`
- Run it: `./pid`.
- Run simulator -> select PID project. Simulator will show control in action

