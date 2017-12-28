#include "vehicle_model.h"

#include <iomanip>

namespace sdcnd_t3p1
{
constexpr double VehicleModel::MAX_STEERING_ANGLE;
constexpr double VehicleModel::MAX_ACCELERATION;
constexpr double VehicleModel::MAX_JERK;
constexpr double VehicleModel::Lf;
constexpr double VehicleModel::STEER_KP;
constexpr double VehicleModel::STEER_KD;
constexpr double VehicleModel::STEER_KI;
constexpr double VehicleModel::SPEED_KP;
constexpr double VehicleModel::SPEED_KD;
constexpr double VehicleModel::SPEED_KI;

//---------------------------------------------------------------------------------------------------------------------
VehicleModel::VehicleModel(double dt)
  : dt_(dt), steeringAngle_(0), speed_(0), lastCte_(0), sumCte_(0), lastOte_(0), sumOte_(0), lastAcceleration_(0), doReset_(true)
//---------------------------------------------------------------------------------------------------------------------
{}

//---------------------------------------------------------------------------------------------------------------------
void VehicleModel::reset(const CartesianPose& pose)
//---------------------------------------------------------------------------------------------------------------------
{
  pose_ = pose;
  steeringAngle_ = 0;
  speed_ = 0;
  lastCte_ = 0;
  sumCte_ = 0;
  lastOte_ = 0;
  sumOte_ = 0;
  lastAcceleration_ = 0;
  doReset_ = true;
}

//---------------------------------------------------------------------------------------------------------------------
CartesianPose VehicleModel::predict(const CartesianPose& refPose, double refSpeed)
//---------------------------------------------------------------------------------------------------------------------
{
  // compute error between vehicle and reference trajectory
  CartesianPose trackingError;
  trackingError.y = refPose.y - pose_.y;
  trackingError.x = refPose.x - pose_.x;

  // compute cross track error
  const double normalToHeading = refPose.heading-M_PI/2;
  const double cte = trackingError.x * cos(normalToHeading) + trackingError.y * sin(normalToHeading);

  // compute on-track error
  const double ote = trackingError.x * cos(refPose.heading) + trackingError.y * sin(refPose.heading);

  std::cout << std::fixed << std::setprecision(3) << std::setfill('0') << std::setw(7)
            << "Error : " << cte << " " << ote << " " << refSpeed - speed_ << std::endl;

  // do the control
  updateControl(refSpeed, ote, cte);
  return move();
}

//---------------------------------------------------------------------------------------------------------------------
CartesianPose VehicleModel::move()
//---------------------------------------------------------------------------------------------------------------------
{
  pose_.x += speed_ * cos(pose_.heading) * dt_;
  pose_.y += speed_ * sin(pose_.heading) * dt_;
  pose_.heading += (speed_/Lf) * steeringAngle_ * dt_;

  return pose_;
}

//---------------------------------------------------------------------------------------------------------------------
void VehicleModel::updateControl(double refSpeed, double ote, double cte)
//---------------------------------------------------------------------------------------------------------------------
{
  if(doReset_)
  {
    lastCte_ = cte;
    lastOte_ = ote;
    doReset_ = false;
  }
  // predicted error. If the control is working, these should eventually go to zero
  const double dcte = cte - lastCte_;
  const double dote = ote - lastOte_;

  steeringAngle_ = -(STEER_KP * cte) - (STEER_KD * dcte) - (STEER_KI * sumCte_);
  steeringAngle_ = std::min(MAX_STEERING_ANGLE, std::max(steeringAngle_, -MAX_STEERING_ANGLE));

  lastCte_ = cte;
  sumCte_ += cte;

  double acceleration = (SPEED_KP * ote) + (SPEED_KD * dote) + (SPEED_KI * sumOte_);
  acceleration = std::min(MAX_ACCELERATION, std::max(acceleration, -MAX_ACCELERATION));

  speed_ += 0.5 * (lastAcceleration_ + acceleration) * dt_;
  lastAcceleration_ = acceleration;
  //speed_ = refSpeed + acceleration * dt_;

  lastOte_ = ote;
  sumOte_ += ote;
}

}
