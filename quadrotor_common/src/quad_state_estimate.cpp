#include "quadrotor_common/quad_state_estimate.h"

#include <string>

#include "quadrotor_common/geometry_eigen_conversions.h"

namespace quadrotor_common
{

QuadStateEstimate::QuadStateEstimate() :
        coordinate_frame(CoordinateFrame::INVALID),
        position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        bodyrates(Eigen::Vector3d::Zero())
{
}

QuadStateEstimate::QuadStateEstimate(
    const nav_msgs::Odometry& state_estimate_msg)
{
  coordinate_frame = CoordinateFrame::INVALID;
  if (state_estimate_msg.header.frame_id.compare("world") == 0)
  {
    coordinate_frame = CoordinateFrame::WORLD;
  }
  else if (state_estimate_msg.header.frame_id.compare("optitrack") == 0)
  {
    coordinate_frame = CoordinateFrame::OPTITRACK;
  }
  else if (state_estimate_msg.header.frame_id.compare("vision") == 0)
  {
    coordinate_frame = CoordinateFrame::VISION;
  }
  else if (state_estimate_msg.header.frame_id.compare("local") == 0)
  {
    coordinate_frame = CoordinateFrame::LOCAL;
  }
  position = geometryToEigen(state_estimate_msg.pose.pose.position);
  velocity = geometryToEigen(state_estimate_msg.twist.twist.linear);
  orientation = geometryToEigen(state_estimate_msg.pose.pose.orientation);
  bodyrates = geometryToEigen(state_estimate_msg.twist.twist.angular);
}

QuadStateEstimate::~QuadStateEstimate()
{
}


void QuadStateEstimate::transformVelocityToWorldFrame()
{
  velocity = orientation * velocity;
}

bool QuadStateEstimate::isValid() const
{
  if (coordinate_frame == CoordinateFrame::INVALID)
  {
    return false;
  }
  if (std::isnan(position.norm()))
  {
    return false;
  }
  if (std::isnan(velocity.norm()))
  {
    return false;
  }
  if (std::isnan(orientation.norm()))
  {
    return false;
  }
  if (std::isnan(bodyrates.norm()))
  {
    return false;
  }

  return true;
}

} // namespace quadrotor_common
