/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "rpg_mpc/mpc_controller.h"
#include <tf/transform_datatypes.h>
#include <ctime>


namespace rpg_mpc {

template<typename T>
MpcController<T>::MpcController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& topic) :
    nh_(nh),
    pnh_(pnh),
    mpc_wrapper_(MpcWrapper<T>()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)),
    est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
                                                  0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
    point_of_interest_(Eigen::Matrix<T, 3, 1>::Zero()) {
  
  nh_.param("mass", mass_, 0.5);

  nh_.param("use_external_yaw", use_external_yaw_, true);

  nh_.param("corrections/kf", kf_correction_, 0.0);
  nh_.param("corrections/r", angle_corrections_[0], 0.0);
  nh_.param("corrections/p", angle_corrections_[1], 0.0);

  nh_.param("gains/rot/x", kR_[0], 1.5);
  nh_.param("gains/rot/y", kR_[1], 1.5);
  nh_.param("gains/rot/z", kR_[2], 1.0);

  nh_.param("gains/ang/x", kOm_[0], 0.13);
  nh_.param("gains/ang/y", kOm_[1], 0.13);
  nh_.param("gains/ang/z", kOm_[2], 0.1);

  pub_predicted_trajectory_ = nh_.advertise<nav_msgs::Path>(topic, 1);
  pub_so3_cmd_              = nh_.advertise<kr_mav_msgs::SO3Command>("mpc/so3_cmd", 1);
  
  
  sub_enable_motors_     = nh_.subscribe("enable_motors", 1,  &MpcController<T>::enableMotorsCallback, this);
  sub_point_of_interest_ = nh_.subscribe("mpc/point_of_interest", 1,  &MpcController<T>::pointOfInterestCallback, this);
  sub_position_cmd_      = nh_.subscribe("mpc/position_cmd", 1,  &MpcController<T>::positionCmdCallback, this);
  sub_odom_              = nh_.subscribe("odom", 1,  &MpcController<T>::odomCallback, this);

  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}

template<typename T>
void MpcController<T>::enableMotorsCallback(
    const std_msgs::Bool::ConstPtr &msg) {

  if(msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;


  // std_msgs::Empty pub_msg;
  // if (enable_motors_ == true) {
  //   start_pub_.publish(pub_msg);
  // } else {
  //   off_pub_.publish(pub_msg);
  // }
}


template<typename T>
void MpcController<T>::pointOfInterestCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
  point_of_interest_(0) = msg->point.x;
  point_of_interest_(1) = msg->point.y;
  point_of_interest_(2) = msg->point.z;
  mpc_wrapper_.setPointOfInterest(point_of_interest_);
}

template<typename T>
void MpcController<T>::odomCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {

  est_state_(kPosX) = msg->pose.pose.position.x;
  est_state_(kPosY) = msg->pose.pose.position.y;
  est_state_(kPosZ) = msg->pose.pose.position.z;
  est_state_(kOriW) = msg->pose.pose.orientation.w;
  est_state_(kOriX) = msg->pose.pose.orientation.x;
  est_state_(kOriY) = msg->pose.pose.orientation.y;
  est_state_(kOriZ) = msg->pose.pose.orientation.z;
  est_state_(kVelX) = msg->twist.twist.linear.x;
  est_state_(kVelY) = msg->twist.twist.linear.y;
  est_state_(kVelZ) = msg->twist.twist.linear.z;

  odom_ = *msg;

}


template<typename T>
void MpcController<T>::positionCmdCallback(
    const kr_mav_msgs::PositionCommand::ConstPtr& msg) {
  
  quadrotor_common::TrajectoryPoint point;

  point.position =  Eigen::Vector3d(msg->position.x, 
                                    msg->position.y, 
                                    msg->position.z);
  point.velocity =  Eigen::Vector3d(msg->velocity.x, 
                                    msg->velocity.y, 
                                    msg->velocity.z);
  point.acceleration =  Eigen::Vector3d(msg->acceleration.x, 
                                        msg->acceleration.y, 
                                        msg->acceleration.z);
  // tf2::Quaternion quat_tf2;
  // quat_tf2.setRPY(0, 0, msg->yaw);
  // geometry_msgs::Quaternion quat_msg;
  // quat_msg = tf2::toMsg(quat_tf2);

  // point.orientation = quat_msg;


  auto q = tf::createQuaternionFromYaw (msg->yaw);
  point.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
  

  point.heading = msg->yaw;
  point.heading_rate = msg->yaw_dot;
  point.heading_acceleration = 0.0f;


  // bodyrates  .yaw_dot
  setReference(quadrotor_common::Trajectory(point));
  run();

}



// template<typename T>
// void MpcController<T>::offCallback(
//     const std_msgs::Empty::ConstPtr& msg) {
//   solve_from_scratch_ = true;
// }

template<typename T>
quadrotor_common::ControlCommand MpcController<T>::off() {
  quadrotor_common::ControlCommand command;

  command.zero();

  return command;
}

template<typename T>
void MpcController<T>::run() {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();

  preparation_thread_.join();

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with hover as initial guess.");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Publish the predicted trajectory.
  publishPrediction(predicted_states_, predicted_inputs_, call_time);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // Return the input control command.
  updateControlCommand(predicted_states_.col(0),
                       predicted_inputs_.col(0),
                       call_time);
  return;
}

// template<typename T>
// bool MpcController<T>::setStateEstimate(
//     const quadrotor_common::QuadStateEstimate& state_estimate_) {
//   est_state_(kPosX) = state_estimate_.position.x();
//   est_state_(kPosY) = state_estimate_.position.y();
//   est_state_(kPosZ) = state_estimate_.position.z();
//   est_state_(kOriW) = state_estimate_.orientation.w();
//   est_state_(kOriX) = state_estimate_.orientation.x();
//   est_state_(kOriY) = state_estimate_.orientation.y();
//   est_state_(kOriZ) = state_estimate_.orientation.z();
//   est_state_(kVelX) = state_estimate_.velocity.x();
//   est_state_(kVelY) = state_estimate_.velocity.y();
//   est_state_(kVelZ) = state_estimate_.velocity.z();
//   const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
//   return quaternion_norm_ok;
// }

template<typename T>
bool MpcController<T>::setReference(
    const quadrotor_common::Trajectory& reference_trajectory_) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  if (reference_trajectory_.points.size() == 1) {
    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        reference_trajectory_.points.front().heading,
        Eigen::Matrix<T, 3, 1>::UnitZ()));
    q_orientation = reference_trajectory_.points.front().orientation.template cast<T>() * q_heading;
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << reference_trajectory_.points.front().position.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        reference_trajectory_.points.front().velocity.template cast<T>()
    ).finished().replicate(1, kSamples + 1);

    acceleration << reference_trajectory_.points.front().acceleration.template cast<T>() - gravity;
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
        reference_trajectory_.points.front().bodyrates.template cast<T>()
    ).finished().replicate(1, kSamples + 1);
  } else {
    auto iterator(reference_trajectory_.points.begin());
    ros::Duration t_start = reference_trajectory_.points.begin()->time_from_start;
    auto last_element = reference_trajectory_.points.end();
    last_element = std::prev(last_element);

    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
             iterator != last_element) {
        iterator++;
      }

      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
      q_orientation = q_heading * iterator->orientation.template cast<T>();
      reference_states_.col(i) << iterator->position.template cast<T>(),
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          iterator->velocity.template cast<T>();
      if (reference_states_.col(i).segment(kOriW, 4).dot(
          est_state_.segment(kOriW, 4)) < 0.0)
        reference_states_.block(kOriW, i, 4, 1) =
            -reference_states_.block(kOriW, i, 4, 1);
      acceleration << iterator->acceleration.template cast<T>() - gravity;
      reference_inputs_.col(i) << acceleration.norm(),
          iterator->bodyrates.template cast<T>();
      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
    }
  }
  return quaternion_norm_ok;
}

template<typename T>
void MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
                                           std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
                                          std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));



  // //update message
  // quadrotor_common::ControlCommand command;

  // command.armed = true;
  // command.control_mode = quadrotor_common::ControlMode::BODY_RATES;

  // command.collective_thrust = input_bounded(INPUT::kThrust);
  // command.bodyrates.x() = input_bounded(INPUT::kRateX);
  // command.bodyrates.y() = input_bounded(INPUT::kRateY);
  // command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  // command.orientation.w() = state(STATE::kOriW);
  // command.orientation.x() = state(STATE::kOriX);
  // command.orientation.y() = state(STATE::kOriY);
  // command.orientation.z() = state(STATE::kOriZ);


  kr_mav_msgs::SO3Command::Ptr so3_cmd =  boost::make_shared<kr_mav_msgs::SO3Command>();

  so3_cmd->header = header_;
  so3_cmd->header.frame_id = odom_.header.frame_id;

  /**
   * Now we need to compute the force. In the SO3 command, the desired force is
   * given in Newtons in the world frame. In the Control command, instead of a
   * force, we get the collective mass normalized thrust (m/s^2). To do the
   * conversion we first multply the mass by the collective thrust to get the
   * desired force in the body frame. Then we transform the force from the body
   * frame to the world frame (intertial frame). f_des = R * [0, 0, u1]^T where
   * R is a rotation matrix representing the orientation of the body frame with
   * respect to the world frame and u1 is the collective thrust.
   */
  tf2::Quaternion quat_tf2;
  tf2::fromMsg(odom_.pose.pose.orientation, quat_tf2);
  // Current orientation of body frame as a rotation matrix
  tf2::Matrix3x3 rot_cur(quat_tf2);
  double u1 = input_bounded(INPUT::kThrust) * mass_;
  so3_cmd->force.x = u1 * rot_cur[0][2];
  so3_cmd->force.y = u1 * rot_cur[1][2];
  so3_cmd->force.z = u1 * rot_cur[2][2];
 
  //std::cout << " so3_cmd->force.z  is" << so3_cmd->force.z << std::endl;

  // Next part of the message is the orientation
  so3_cmd->orientation.w = state(STATE::kOriW);
  so3_cmd->orientation.x = state(STATE::kOriX);
  so3_cmd->orientation.y = state(STATE::kOriY);
  so3_cmd->orientation.z = state(STATE::kOriZ);


  // Need to check if the angular velocity should be in body frame or world
  // frame. Currently considering it to be in body frame
  so3_cmd->angular_velocity.x = input_bounded(INPUT::kRateX);
  so3_cmd->angular_velocity.y = input_bounded(INPUT::kRateY);
  so3_cmd->angular_velocity.z = input_bounded(INPUT::kRateZ);


  // Fill the aux portion
  so3_cmd->aux.current_yaw = tf::getYaw(odom_.pose.pose.orientation);
  so3_cmd->aux.kf_correction = kf_correction_;
  so3_cmd->aux.angle_corrections[0] = angle_corrections_[0];
  so3_cmd->aux.angle_corrections[1] = angle_corrections_[1];
  so3_cmd->aux.enable_motors = enable_motors_;
  so3_cmd->aux.use_external_yaw = use_external_yaw_;

  // // Fill the kR and kOm gains
  so3_cmd->kR[0] = kR_[0];
  so3_cmd->kR[1] = kR_[1];
  so3_cmd->kR[2] = kR_[2];
  so3_cmd->kOm[0] = kOm_[0];
  so3_cmd->kOm[1] = kOm_[1];
  so3_cmd->kOm[2] = kOm_[2];

  // Finally publish the message
  pub_so3_cmd_.publish(so3_cmd);


  return;
}

template<typename T>
bool MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
    ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  T dt = mpc_wrapper_.getTimestep();

  for (int i = 0; i < kSamples; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i; 
    pose.pose.position.x = states(kPosX, i);
    pose.pose.position.y = states(kPosY, i);
    pose.pose.position.z = states(kPosZ, i);
    pose.pose.orientation.w = states(kOriW, i);
    pose.pose.orientation.x = states(kOriX, i);
    pose.pose.orientation.y = states(kOriY, i);
    pose.pose.orientation.z = states(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  pub_predicted_trajectory_.publish(path_msg);

  return true;
}


template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_,
      params.max_bodyrate_xy_, params.max_bodyrate_z_);
  mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template
class MpcController<float>;

template
class MpcController<double>;

} // namespace rpg_mpc
