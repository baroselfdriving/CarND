#ifndef BEHAVIOUR_PREDICTOR_H
#define BEHAVIOUR_PREDICTOR_H

#include "vehicle.h"
#include <map>

namespace sdcnd_t3p1
{

class BehaviourPredictor
{
public:
  static constexpr double PREDICTION_TIME = 5;

public:
  struct Prediction
  {
    int laneNumber;
    double freeDistance; //!< available manouvre distance on this lane
    double laneSpeed;
  };

  using LanePredictionMap = std::map<int/*lane*/, Prediction>;

public:
  BehaviourPredictor() = default;
  ~BehaviourPredictor() = default;

  LanePredictionMap predict(const Vehicle& me, const VehicleList& others);

private:

};

}
#endif // BEHAVIOUR_PREDICTOR_H

