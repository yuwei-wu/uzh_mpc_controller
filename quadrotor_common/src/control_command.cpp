#include "quadrotor_common/control_command.h"

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

ControlCommand::ControlCommand() :
    control_mode(ControlMode::NONE), armed(false),
        orientation(Eigen::Quaterniond::Identity()), bodyrates(
        Eigen::Vector3d::Zero()), angular_accelerations(
        Eigen::Vector3d::Zero()), collective_thrust(0.0), rotor_thrusts()
{
}


ControlCommand::~ControlCommand()
{
}

void ControlCommand::zero()
{
  control_mode = ControlMode::BODY_RATES;
  armed = false;
  orientation = Eigen::Quaterniond::Identity();
  bodyrates = Eigen::Vector3d::Zero();
  angular_accelerations = Eigen::Vector3d::Zero();
  collective_thrust = 0.0;
  rotor_thrusts = Eigen::VectorXd::Zero(rotor_thrusts.size());
}



} // namespace quadrotor_common
