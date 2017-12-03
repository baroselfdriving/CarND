# Behaviour Planning - Term 3, Lesson 4

## Implement a Behavior Planner

In this exercise you will implement a behavior planner and cost functions for highway driving. The planner will use prediction data to set the state of the ego vehicle to one of 5 values and generate a corresponding vehicle trajectory:

- "KL" - Keep Lane
- "LCL" / "LCR"- Lane Change Left / Lane Change Right
- "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right

The objective of the quiz is to navigate through traffic to the goal in as little time as possible. Note that the goal lane and s value, as well as the traffic speeds for each lane, are set in main.cpp below. Since the goal is in the slowest lane, in order to get the lowest time, you'll want to choose cost functions and weights to drive in faster lanes when appropriate. We've provided two suggested cost functions in cost.cpp.

## Instructions

- Implement the choose_next_state method in the vehicle.cpp class. You can use the Behavior Planning Pseudocode concept as a guideline, although in this quiz, there are a couple of small differences: you'll be returning a best trajectory corresponding to the best state instead of the state itself. Additionally, the function inputs will be slightly different in this quiz than in the classroom concept.

- Implement cost functions in cost.cpp file.

- Choose appropriate weights to induce the desired vehicle behavior.
- Hit Test Run and see how your car does! How fast can you get to the goal?

## Extra Practice
Provided in one of the links below is a zip file python_3_practice, which is the same problem written in Python 3 - you can optionally use this file for additional coding practice. In the python_3_solution link, the solution is provided as well. If you get stuck on the quiz see if you can convert the python solution to C++ and pass the classroom quiz with it. You can run the python quiz with python simulate_behavior.py.
