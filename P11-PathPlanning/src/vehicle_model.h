#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "vehicle.h"

#include <cmath>
#include <iostream>

namespace sdcnd_t3p1
{

/// Provides a bicycle model for kinematics of the autonomous car
class VehicleModel
{
public:
  static constexpr double Lf = 2.67; //!< Distance from CoM to front axle. Using value from SDCND Term 2 MPC project
  static constexpr double MAX_STEERING_ANGLE = 25 * M_PI/180.;
  static constexpr double GAIN_KP = 0.1; //!< proportional gain for steering angle controller
  static constexpr double GAIN_KD = 1.5; //!< differential gain for steering angle controller
  static constexpr double GAIN_KI = 0.0001; //!< integral gain for steering angle controller

public:
  VehicleModel(double dt) : steeringAngle_(0), dt_(dt), lastCrossTrackErr_(0), sumCte_(0) {}

  ~VehicleModel() = default;

  /// reset model variables
  void reset(const CartesianPose& pose)
  {
    pose_ = pose;
    steeringAngle_ = 0;
    lastCrossTrackErr_ = 0;
    sumCte_ = 0;
  }

  /// get current position of vehicle
  const CartesianPose& getPose() const { return pose_; }

  /// predict motion for next time step
  CartesianPose predict(double crossTrackError, double speed)
  {
    updateSteeringAngle(crossTrackError, speed);
    return move(speed);
  }

private:
  /// Simulate motion for a time slice, given current position and forward speed
  CartesianPose move(double speed)
  {
    pose_.x += speed * cos(pose_.heading) * dt_;
    pose_.y += speed * sin(pose_.heading) * dt_;
    pose_.heading += (speed/Lf) * steeringAngle_ * dt_;

    //std::cout << "P: " << pose_.x << " " << pose_.y << " "<< pose_.heading *180/M_PI << std::endl;

    return pose_;
  }

  /// Apply PID control and generate a steering angle update, given cross track error
  void updateSteeringAngle(double cte, double speed)
  {
    // predicted error. If the control is working, these should eventually go to zero
    const double dcte = cte - lastCrossTrackErr_;

    steeringAngle_ = -(GAIN_KP * cte) - (GAIN_KD * dcte) - (GAIN_KI * sumCte_);
    if(fabs(steeringAngle_) > MAX_STEERING_ANGLE)
    {
      std::cout << "E: " << cte << ", " << dcte << ", " << GAIN_KP * cte << ", " << GAIN_KD * dcte << std::endl;
    }
    steeringAngle_ = std::min(MAX_STEERING_ANGLE, std::max(steeringAngle_, -MAX_STEERING_ANGLE));

    lastCrossTrackErr_ = cte;
    sumCte_ += cte;
  }

private:
  CartesianPose pose_;
  double steeringAngle_;
  double dt_; //!< control period
  double lastCrossTrackErr_;
  double sumCte_;
};

}

#endif // VEHICLE_MODEL_H
