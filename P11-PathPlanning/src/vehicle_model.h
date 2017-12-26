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
  static constexpr double GAIN_KP = 0;//0.01; //!< proportional gain for steering angle controller
  static constexpr double GAIN_KD = 3;//1.2;  //!< differential gain for steering angle controller
  static constexpr double GAIN_KI = 0;//0.001; //!< integral gain for steering angle controller

public:
  VehicleModel(double dt) : steeringAngle_(0), dt_(dt), lastCrossTrackErr_(0), lastHeadingErr_(0), sumCte_(0) {}

  ~VehicleModel() = default;

  /// reset model variables
  void reset(const CartesianPose& pose)
  {
    pose_ = pose;
    steeringAngle_ = 0;
    lastCrossTrackErr_ = 0;
    lastHeadingErr_ = 0;
    sumCte_ = 0;
  }

  /// get current position of vehicle
  const CartesianPose& getPose() const { return pose_; }

  /// predict motion for next time step
  CartesianPose predict(double crossTrackError, double headingError, double speed)
  {
    updateSteeringAngle(crossTrackError, headingError, speed);
    return move(speed);
  }

private:
  /// Simulate motion for a time slice, given current position and forward speed
  CartesianPose move(double speed)
  {
    pose_.x += speed * cos(pose_.heading) * dt_;
    pose_.y += speed * sin(pose_.heading) * dt_;
    pose_.heading += (speed/Lf) * steeringAngle_ * dt_;
    return pose_;
  }

  /// Apply PID control and generate a steering angle update, given cross track error
  void updateSteeringAngle(double cte, double he, double speed)
  {
    // predicted error. If the control is working, these should eventually go to zero
    double cteNext = cte + speed * sin(he) * dt_;
    double heNext = he + (speed/Lf) * steeringAngle_ * dt_;

    std::cout << "E: " << cteNext << " " << heNext << std::endl;

    steeringAngle_ = -GAIN_KP * cte - GAIN_KD * (cte - lastCrossTrackErr_) - GAIN_KI * sumCte_;
    lastCrossTrackErr_ = cte;
    lastHeadingErr_ = he;
    sumCte_ += cte;
  }

private:
  CartesianPose pose_;
  double steeringAngle_;
  double dt_; //!< control period
  double lastCrossTrackErr_;
  double lastHeadingErr_;
  double sumCte_;
};

}

#endif // VEHICLE_MODEL_H
