#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "vehicle.h"

#include <cmath>
#include <iostream>

namespace sdcnd_t3p1
{

/// Provides a bicycle model for kinematics of the autonomous car. Also encodes a
/// low level speed and steering controller
class VehicleModel
{
public:
  static constexpr double Lf = 2.67; //!< Distance from CoM to front axle. Using value from SDCND Term 2 MPC project
  static constexpr double MAX_STEERING_ANGLE = 25 * M_PI/180.;
  static constexpr double MAX_ACCELERATION = 7;
  static constexpr double MAX_JERK = 10;

  static constexpr double STEER_KP = 0.05; //!< proportional gain for steering angle controller
  static constexpr double STEER_KD = 0.9; //!< differential gain for steering angle controller
  static constexpr double STEER_KI = 0;//0.0001; //!< integral gain for steering angle controller

  static constexpr double SPEED_KP = 3; //!< proportional gain for speed controller
  static constexpr double SPEED_KD = 200; //!< differential gain for speed controller
  static constexpr double SPEED_KI = 0;//0; //!< integral gain for speed controller

public:
  VehicleModel(double dt);

  ~VehicleModel() = default;

  /// reset model variables
  void reset(const CartesianPose& pose);

  /// get current position of vehicle
  const CartesianPose& getPose() const { return pose_; }

  /// predict motion for next time step
  CartesianPose predict(const CartesianPose& refPose, double refSpeed);

private:
  /// Simulate motion for a time slice
  CartesianPose move();

  /// Apply PID control and generate a steering angle and acceleration updates, given
  /// cross track and on-track errors
  void updateControl(double refSpeed, double ote, double cte);

private:
  double dt_; //!< control period
  double steeringAngle_;
  double speed_;
  CartesianPose pose_;
  double lastCte_;
  double sumCte_;
  double lastOte_;
  double sumOte_;
  double lastAcceleration_;
  bool doReset_;
};

}

#endif // VEHICLE_MODEL_H
