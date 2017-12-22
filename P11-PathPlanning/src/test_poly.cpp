#include "trajectory_planner.h"

#include <iostream>
#include <deque>

//---------------------------------------------------------------------------------------------------------------------
int main()
//---------------------------------------------------------------------------------------------------------------------
{
  std::deque<TrajectoryPlanner::FrenetState> history;

  int nPoints = 500;
  double timeDelta = 10;
  double longSpeed = 25;
  double latPos = 6;

  TrajectoryPlanner::generateFrenetWaypoints(longSpeed, latPos, timeDelta, nPoints, history);

  nPoints = 250;
  history.erase(history.begin(), history.begin()+nPoints);
  TrajectoryPlanner::generateFrenetWaypoints(longSpeed, latPos, timeDelta, nPoints, history);

  return 0;
}
