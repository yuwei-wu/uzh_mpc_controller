#include "quadrotor_common/trajectory.h"

#include <Eigen/Dense>

#include "quadrotor_common/math_common.h"

namespace quadrotor_common
{

Trajectory::Trajectory() :
  trajectory_type(TrajectoryType::UNDEFINED),
  points()
{
}


Trajectory::Trajectory(const quadrotor_common::TrajectoryPoint& point) :
  trajectory_type(TrajectoryType::GENERAL),
  points()
{
  points.push_back(point);
}

Trajectory::~Trajectory()
{
}


nav_msgs::Path Trajectory::toRosPath() const
{
  nav_msgs::Path path_msg;
  ros::Time t = ros::Time::now();
  path_msg.header.stamp = t;
  path_msg.header.frame_id = "world";

  geometry_msgs::PoseStamped pose;

  std::list<quadrotor_common::TrajectoryPoint>::const_iterator it;
  for (it = points.begin(); it != points.end(); it++)
  {
    pose.header.stamp = t + it->time_from_start;
    pose.pose.position.x = it->position.x();
    pose.pose.position.y = it->position.y();
    pose.pose.position.z = it->position.z();
    pose.pose.orientation.w = it->orientation.w();
    pose.pose.orientation.x = it->orientation.x();
    pose.pose.orientation.y = it->orientation.y();
    pose.pose.orientation.z = it->orientation.z();
    path_msg.poses.push_back(pose);
  }

  return path_msg;
}

quadrotor_common::TrajectoryPoint Trajectory::getStateAtTime(
  const ros::Duration& time_from_start) const
{
  if (time_from_start <= points.front().time_from_start)
  {
    return points.front();
  }
  if (time_from_start >= points.back().time_from_start)
  {
    return points.back();
  }

  quadrotor_common::TrajectoryPoint trajectory_point;

  // Find points p0 and p1 such that
  // p0.time_from_start <= time_from_start <= p1.time_from_start
  std::list<quadrotor_common::TrajectoryPoint>::const_iterator p1;
  for (p1 = points.begin(); p1 != points.end(); p1++)
  {
    if (p1->time_from_start > time_from_start)
    {
      break;
    }
  }
  std::list<quadrotor_common::TrajectoryPoint>::const_iterator p0 = std::prev(
    p1);
  const double interp_ratio = (time_from_start - p0->time_from_start).toSec()
    / (p1->time_from_start - p0->time_from_start).toSec();

  return interpolate(*p0, *p1, interp_ratio);
}

} // namespace quadrotor_common
