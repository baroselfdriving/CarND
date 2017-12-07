# Implementing Hybrid A*

In this exercise, you will be provided a working implementation of a breadth first search algorithm which does not use
any heuristics to improve its efficiency. Your goal is to try to make the appropriate modifications to the algorithm so
that it takes advantage of heuristic functions (possibly the ones mentioned in the previous paper) to reduce the number
of grid cell expansions required.

## Instructions:
Modify the code in 'hybrid_breadth_first.cpp' and hit Test Run to check your results.
Note the number of expansions required to solve an empty 15x15 grid (it should be about 18,000!). Modify the code to
try to reduce that number. How small can you get it?

## Solution
Here is one possible implementation for Hybrid A* using the "distance to goal" heuristic function. In this
implementation, we have added an f value to the maze_s struct, which is set in the expand function. Additionally,
we've added two new functions: heuristic and compare_maze_s. The compare_maze_s function is used for comparison of
maze_s objects when sorting the opened stack.

Another possible is to use the regular A* algorithm to assign a cost value to each grid cell. This grid of costs can
then be used as the heuristic function. If you are looking for additional practice, give this a try!
