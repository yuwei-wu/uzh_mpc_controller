#pragma once

#include <list>
#include <nav_msgs/Path.h>
#include <ros/time.h>

#include "quadrotor_common/trajectory_point.h"

namespace quadrotor_common
{

struct Trajectory
{
  Trajectory();
  Trajectory(const quadrotor_common::TrajectoryPoint& point);
  virtual ~Trajectory();

  nav_msgs::Path toRosPath() const;
  quadrotor_common::TrajectoryPoint getStateAtTime(
    const ros::Duration& time_from_start) const;

  enum class TrajectoryType
  {
    UNDEFINED, GENERAL, ACCELERATION, JERK, SNAP
  } trajectory_type;

  std::list<quadrotor_common::TrajectoryPoint> points;
};

} // namespace quadrotor_common
