# Project 9: PID Control
Self-Driving Car Engineer Nanodegree Program

---
## Introduction
This project implements a PID controller to manouvre a vehicle in the simulated race track. The simulator provided the
cross track error (CTE) and the velocity (mph), and the PID controller is meant to generate the appropriate steering
angle to keep the vehicle within the edges of the track. The next section describes how I achieved controller tuning.

## Discussion
### Describe the effect each of the P, I, D components had in your implementation.

The proportional (P) controller is responsible for generating most of the control 'effort' to get the car on to the
desired trajectory. A higher P gain would make the car get to the trajectory quicker, but will also cause the car to
overshoot the trajectory by a larger amount.

The derivative (D) controller counter-steers as the car gets closer to the desired trajectory, reducing the overshoot.
A larger D gain dampens the oscillatory behaviour caused by the proportional controller alone.

The integral (I) controller is there to reduce the steady state offset from the desired trajectory caused by systematic
errors such as steering offset (misaligned wheels). The effect of this controller is rather subtle in the simulated
race track because its curviness. On a straight road, I suppose any steady-state offsets would have been more noticeable.

### Describe how the final hyperparameters were chosen.

A basic PID controller was first implemented using the code framework provided. I hand-tuned the control gains to
achieve stable vehicle control in the simulator. There was some swaying, but the vehicle remained within the boundaries
of the track. The trottle was set to preset default of 0.3. The hand-tuned PID gains were [.15, 0.001, 1] respectively.
[Here's the video of the simulated vehicle running with these PID parameters](https://youtu.be/Q3lQnczC0h0).

Once I hand-tuned PID gains to the above, I implemented a naive version of the twiddle algorithm to see if I can
autotune the gains further. This was tricky, to put it mildly. Here's a brief description of how I did it.

- First, the hand-tuned initial set of gain parameters must be good to begin with - at least within the right order of magnitude.

- The auto-tuning algorithm must tune each gain and evaluate their effect one at a time before moving on to the next
one; i.e. adjust P gain, evaluate what it did to the error (and adjust the gain for the next cycle), then do the same
for I gain and then D gain.

- The curviness of the simulated race track and the trottle (max speed) has a bearing on the number of control steps
required to evaluate the effect of a gain change. Tuning the gains would have been easy if this was a regulation
problem (ie. the desired velocity is a constant) rather than a tracking problem (velocity vector changes constantly
because we are tracking the center line of a curvy road). The number of control loops to evaluate effectiveness of a
set of gains is a hyperparameter(`SetNumTwiddleEvalSamples()` in the code. For smoother control changes, this number
should be large (~100) so that we can evaluate the error over a sufficient length of the simulated race track. However,
if it is too large the car may have gone off the road before a control change kicks in. If it is too small, the control
changes may happen too often, causing the car to zig-zag. After much experimentation, I set this to 50 samples.

- Having run the simulation successfully with the above set of parameters, I got a bit brave and decided to increase
the trottle up from 0.3 (preset default, max speed < 40 mph) in steps. 0.4...good (max speed ~44mph). 0.5...good
(max speed ~56 mph). 0.6...crash. So there you go! I couldn't really get it working at trottle > 0.5.

- For trottle = 0.3, the PID control gains eventually settle down to about [0.11, 0.001, 1.2] respectively. [Here's
the video of the simulated car doing it's thing](https://youtu.be/xTGGPnw2vfQ).

## Build Instructions

- Clone this repo.
- Install websockets library: `sh ./install-ubuntu.sh`
- Make a build directory: `mkdir build && cd build`
- Compile: `cmake .. && make`
- Run it: `./pid`.
- Run simulator -> select PID project. Simulator will show control in action

