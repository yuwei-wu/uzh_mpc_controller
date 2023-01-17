#include "quadrotor_common/trajectory_point.h"

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

TrajectoryPoint::TrajectoryPoint() :
    time_from_start(ros::Duration(0.0)), position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3d::Zero()), acceleration(
        Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), snap(
        Eigen::Vector3d::Zero()), bodyrates(Eigen::Vector3d::Zero()),
        angular_acceleration(Eigen::Vector3d::Zero()),
        angular_jerk(Eigen::Vector3d::Zero()), angular_snap(
        Eigen::Vector3d::Zero()), heading(0.0), heading_rate(0.0),
        heading_acceleration(0.0)
{
}

TrajectoryPoint::~TrajectoryPoint()
{
}

} // namespace quadrotor_common
