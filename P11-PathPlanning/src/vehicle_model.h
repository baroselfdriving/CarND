#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "vehicle.h"

#include <cmath>

namespace sdcnd_t3p1
{

/// Provides a bicycle model for kinematics of the autonomous car
class VehicleModel
{
public:
  static constexpr double Lf = 2.67; //!< Distance from CoM to front axle. Using value from SDCND Term 2 MPC project
  static constexpr double MAX_STEERING_ANGLE = 25 * M_PI/180.;
  static constexpr double GAIN_KP = 0.11; //!< proportional gain for steering angle controller
  static constexpr double GAIN_KD = 1.2;  //!< differential gain for steering angle controller
  static constexpr double GAIN_KI = 0.001; //!< integral gain for steering angle controller

public:
  VehicleModel(double dt) : steeringAngle_(0), dt_(dt) {}

  ~VehicleModel() = default;

  /// reset model variables
  void reset()
  {
    steeringAngle_ = 0;
    lastCte_ = 0;
    sumCte_ = 0;
  }

  /// predict motion for next time step
  CartesianPose predict(const CartesianPose& pose, double cte, double speed)
  {
    updateSteeringAngle(cte);
    return move(pose, speed);
  }

private:
  /// Simulate motion for a time slice, given current position and forward speed
  CartesianPose move(const CartesianPose& pose, double speed)
  {
    CartesianPose p;
    p.x = pose.x + speed * cos(pose.heading) * dt_;
    p.y = pose.y + speed * sin(pose.heading) * dt_;
    p.heading = pose.heading + (speed/Lf) * steeringAngle_ * dt_;
    return p;
  }

  /// Apply PID control and generate a steering angle update, given cross track error
  double updateSteeringAngle(double cte)
  {
    double steeringAngle = -GAIN_KP * cte - GAIN_KD * (cte - lastCte_) - GAIN_KI * sumCte_;
    sumCte_ += cte;
    return steeringAngle;
  }

private:
  double steeringAngle_;
  double dt_; //!< control period
  double lastCte_;
  double sumCte_;
};

}

#endif // VEHICLE_MODEL_H
