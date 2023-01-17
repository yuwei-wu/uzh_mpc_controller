#pragma once

#include <Eigen/Dense>

namespace quadrotor_common
{

enum class ControlMode
{
  NONE, ATTITUDE, BODY_RATES, ANGULAR_ACCELERATIONS, ROTOR_THRUSTS
};

struct ControlCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControlCommand();
  virtual ~ControlCommand();

  void zero();

  // Control mode as defined above
  ControlMode control_mode;

  // Flag whether controller is allowed to arm
  bool armed;

  // Orientation of the body frame with respect to the world frame
  Eigen::Quaterniond orientation; // [-]

  // Body rates in body frame
  // Note that in ATTITUDE mode the x-y-bodyrates are only feed forward terms 
  // computed from a reference trajectory
  // Also in ATTITUDE mode, the z-bodyrate has to be from feedback control
  Eigen::Vector3d bodyrates; // [rad/s]

  // Angular accelerations in body frame
  Eigen::Vector3d angular_accelerations; // [rad/s^2]

  // Collective mass normalized thrust
  double collective_thrust; // [m/s^2]

  // Single rotor thrusts [N]
  // These are only considered in the ROTOR_THRUSTS control mode
  Eigen::VectorXd rotor_thrusts;
};

} // namespace quadrotor_common
