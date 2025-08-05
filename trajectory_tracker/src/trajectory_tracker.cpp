/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
   Original idea of the implemented control scheme was published on:
   S. Iida, S. Yuta, "Vehicle command system and trajectory control for
   autonomous mobile robots," in Proceedings of the 1991 IEEE/RSJ
   International Workshop on Intelligent Robots and Systems (IROS),
   1991, pp. 212-217.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>
#include <trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp>

#include <trajectory_tracker/basic_control.h>
#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/path2d.h>

namespace trajectory_tracker
{
class TrackerNode: public rclcpp::Node
{
public:
  TrackerNode();
  ~TrackerNode();
  void spin();

private:
  std::string topic_path_;
  std::string topic_cmd_vel_;
  std::string frame_robot_;
  std::string frame_odom_;
  double hz_;
  double look_forward_;
  double curv_forward_;
  double k_[3];
  double gain_at_vel_;
  double d_lim_;
  double d_stop_;
  double vel_[2];
  double acc_[2];
  double acc_toc_[2];
  trajectory_tracker::VelAccLimitter v_lim_;
  trajectory_tracker::VelAccLimitter w_lim_;
  double rotate_ang_;
  double goal_tolerance_dist_;
  double goal_tolerance_ang_;
  double stop_tolerance_dist_;
  double stop_tolerance_ang_;
  double no_pos_cntl_dist_;
  double min_track_path_;
  int path_step_;
  int path_step_done_;
  bool allow_backward_;
  bool limit_vel_by_avel_;
  bool check_old_path_;
  double epsilon_;
  double max_dt_;
  bool use_time_optimal_control_;
  double time_optimal_control_future_gain_;
  double k_ang_rotation_;
  double k_avel_rotation_;
  double goal_tolerance_lin_vel_;
  double goal_tolerance_ang_vel_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<trajectory_tracker_msgs::msg::PathWithVelocity>::SharedPtr sub_path_velocity_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tracking_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::TimerBase::SharedPtr odom_timeout_timer_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_hander_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> callback_handles_;
  double odom_timeout_sec_;

  trajectory_tracker::Path2D path_;
  std_msgs::msg::Header path_header_;
  bool is_path_updated_;

  bool use_odom_;
  bool predict_odom_;
  rclcpp::Time prev_odom_stamp_;

  struct TrackingResult
  {
    explicit TrackingResult(const int s)
      : status(s)
      , distance_remains(0.0)
      , angle_remains(0.0)
      , distance_remains_raw(0.0)
      , angle_remains_raw(0.0)
      , turning_in_place(false)
      , signed_local_distance(0.0)
      , distance_from_target(0.0)
      , target_linear_vel(0.0)
      , tracking_point_x(0.0)
      , tracking_point_y(0.0)
      , tracking_point_curv(0.0)
      , path_step_done(0)
    {
    }

    int status;  // same as trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::status
    double distance_remains;
    double angle_remains;
    double distance_remains_raw;  // remained distance without prediction
    double angle_remains_raw;
    bool turning_in_place;
    double signed_local_distance;
    double distance_from_target;
    double target_linear_vel;
    double tracking_point_x;
    double tracking_point_y;
    double tracking_point_curv;
    int path_step_done;
  };

  template <typename MSG_TYPE>
  void cbPath(const typename MSG_TYPE::ConstPtr&);
  void cbSpeed(const std_msgs::msg::Float32::ConstPtr&);
  void cbOdometry(const nav_msgs::msg::Odometry::ConstPtr&);
  void cbTimer();
  void cbOdomTimeout();
  void control(const tf2::Stamped<tf2::Transform>&, const Eigen::Vector3d&, const double, const double, const double);
  TrackingResult getTrackingResult(
      const tf2::Stamped<tf2::Transform>&, const Eigen::Vector3d&, const double, const double);
};

TrackerNode::TrackerNode()
  : rclcpp::Node("trajectory_tracker")
  , is_path_updated_(false)
  , prev_odom_stamp_(0LL, RCL_ROS_TIME)
{
  frame_robot_ = this->declare_parameter<std::string>("frame_robot", "base_link");
  frame_odom_ = this->declare_parameter<std::string>("frame_odom", "odom");
  topic_path_ = this->declare_parameter<std::string>("path", "path");
  topic_cmd_vel_ = this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  hz_ = this->declare_parameter<double>("hz", 50.0);
  use_odom_ = this->declare_parameter<bool>("use_odom", false);
  predict_odom_ = this->declare_parameter<bool>("predict_odom", true);
  max_dt_ = this->declare_parameter<double>("max_dt", 0.1);
  odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 0.1);
  look_forward_ = this->declare_parameter<double>("look_forward", 0.5);
  curv_forward_ = this->declare_parameter<double>("curv_forward", 0.5);
  k_[0]=this->declare_parameter<double>("k_dist", 1.0);
  k_[1]=this->declare_parameter<double>("k_ang", 1.0);
  k_[2]=this->declare_parameter<double>("k_avel", 1.0);
  gain_at_vel_=this->declare_parameter<double>("gain_at_vel", 0.0);
  d_lim_=this->declare_parameter<double>("dist_lim", 0.5);
  d_stop_=this->declare_parameter<double>("dist_stop", 2.0);
  rotate_ang_=this->declare_parameter<double>("rotate_ang", 0.78539816339);
  vel_[0]=this->declare_parameter<double>("max_vel", 0.5);
  vel_[1]=this->declare_parameter<double>("max_angvel", 1.0);
  acc_[0]=this->declare_parameter<double>("max_acc", 1.0);
  acc_[1]=this->declare_parameter<double>("max_angacc", 2.0);
  acc_toc_[0]=acc_[0]*this->declare_parameter<double>("acc_toc_factor", 0.9);
  acc_toc_[1]=acc_[1]* this->declare_parameter<double>("angacc_toc_factor", 0.9);
  path_step_ = this->declare_parameter<int>("path_step", 1);
  goal_tolerance_dist_ = this->declare_parameter<double>("goal_tolerance_dist", 0.2);
  goal_tolerance_ang_ = this->declare_parameter<double>("goal_tolerance_ang", 0.1);
  stop_tolerance_dist_ = this->declare_parameter<double>("stop_tolerance_dist", 0.1);
  stop_tolerance_ang_ = this->declare_parameter<double>("stop_tolerance_ang", 0.05);
  no_pos_cntl_dist_ = this->declare_parameter<double>("no_position_control_dist", 0.0);
  min_track_path_ = this->declare_parameter<double>("min_tracking_path", no_pos_cntl_dist_);
  allow_backward_ = this->declare_parameter<bool>("allow_backward", true);
  limit_vel_by_avel_ = this->declare_parameter<bool>("limit_vel_by_avel", false);
  check_old_path_ = this->declare_parameter<bool>("check_old_path", false);
  epsilon_ = this->declare_parameter<double>("epsilon", 0.001);
  use_time_optimal_control_ = this->declare_parameter<bool>("use_time_optimal_control", true);
  time_optimal_control_future_gain_ = this->declare_parameter<double>("time_optimal_control_future_gain", 1.5);
  k_ang_rotation_ = this->declare_parameter<double>("k_ang_rotation", 1.0);
  k_avel_rotation_ = this->declare_parameter<double>("k_avel_rotation", 1.0);
  goal_tolerance_lin_vel_ = this->declare_parameter<double>("goal_tolerance_lin_vel", 0.0);
  goal_tolerance_ang_vel_ = this->declare_parameter<double>("goal_tolerance_ang_vel", 0.0);

  using std::placeholders::_1;
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", 2, std::bind(&TrackerNode::cbPath<nav_msgs::msg::Path>, this, _1));
  sub_path_velocity_ = this->create_subscription<trajectory_tracker_msgs::msg::PathWithVelocity>(
    "path_velocity", 2, std::bind(&TrackerNode::cbPath<trajectory_tracker_msgs::msg::PathWithVelocity>, this, _1));
  sub_vel_ = this->create_subscription<std_msgs::msg::Float32>(
    "speed", 20, std::bind(&TrackerNode::cbSpeed, this, _1));
  pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pub_status_ = this->create_publisher<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>("~/status", rclcpp::QoS(10).transient_local());
  pub_tracking_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/tracking", rclcpp::QoS(10).transient_local());
  if (use_odom_)
  {
    // ! ROS 2 cannot specity TCP_NoDelay for a topic
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&TrackerNode::cbOdometry, this, _1));
  }

  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

  parameter_hander_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  std::map<std::string, std::function<void(const rclcpp::Parameter&)>> on_parameter_map = {
      {"look_forward", [this](const rclcpp::Parameter& p) { look_forward_ = p.as_double(); }},
      {"curv_forward", [this](const rclcpp::Parameter& p) { curv_forward_ = p.as_double(); }},
      {"k_dist", [this](const rclcpp::Parameter& p) {k_[0]=p.as_double();}},
      {"k_ang", [this](const rclcpp::Parameter& p) {k_[1]=p.as_double();}},
      {"k_avel", [this](const rclcpp::Parameter& p) {k_[2]=p.as_double();}},
      {"gain_at_vel", [this](const rclcpp::Parameter& p) {gain_at_vel_=p.as_double();}},
      {"dist_lim", [this](const rclcpp::Parameter& p) {d_lim_=p.as_double();}},
      {"dist_stop", [this](const rclcpp::Parameter& p) {d_stop_=p.as_double();}},
      {"rotate_ang", [this](const rclcpp::Parameter& p) {rotate_ang_=p.as_double();}},
      {"max_vel", [this](const rclcpp::Parameter& p) {vel_[0]=p.as_double();}},
      {"max_angvel", [this](const rclcpp::Parameter& p) {vel_[1]=p.as_double();}},
      {"max_acc", [this](const rclcpp::Parameter& p) {acc_[0]=p.as_double();acc_toc_[0]=acc_[0]*p.as_double();}},
      {"max_angacc", [this](const rclcpp::Parameter& p) {acc_[1]=p.as_double();acc_toc_[1]=acc_[1]* p.as_double();}},
      {"acc_toc_factor", [this](const rclcpp::Parameter& p) {acc_toc_[0]=acc_[0]*p.as_double();}},
      {"angacc_toc_factor", [this](const rclcpp::Parameter& p) {acc_toc_[1]=acc_[1]* p.as_double();}},
      {"path_step", [this](const rclcpp::Parameter& p) {path_step_ = p.as_int();}},
      {"goal_tolerance_dist", [this](const rclcpp::Parameter& p) {goal_tolerance_dist_ = p.as_double();}},
      {"goal_tolerance_ang", [this](const rclcpp::Parameter& p) {goal_tolerance_ang_ = p.as_double();}},
      {"stop_tolerance_dist", [this](const rclcpp::Parameter& p) {stop_tolerance_dist_ = p.as_double();}},
      {"stop_tolerance_ang", [this](const rclcpp::Parameter& p) {stop_tolerance_ang_ = p.as_double();}},
      {"no_position_control_dist", [this](const rclcpp::Parameter& p) {no_pos_cntl_dist_ = p.as_double();}},
      {"min_tracking_path", [this](const rclcpp::Parameter& p) {min_track_path_ = p.as_double();}},
      {"allow_backward", [this](const rclcpp::Parameter& p) {allow_backward_ = p.as_bool();}},
      {"limit_vel_by_avel", [this](const rclcpp::Parameter& p) {limit_vel_by_avel_ = p.as_bool();}},
      {"check_old_path", [this](const rclcpp::Parameter& p) {check_old_path_ = p.as_bool();}},
      {"epsilon", [this](const rclcpp::Parameter& p) {epsilon_ = p.as_double();}},
      {"use_time_optimal_control", [this](const rclcpp::Parameter& p) {use_time_optimal_control_ = p.as_bool();}},
      {"time_optimal_control_future_gain", [this](const rclcpp::Parameter& p) {time_optimal_control_future_gain_ = p.as_double();}},
      {"k_ang_rotation", [this](const rclcpp::Parameter& p) {k_ang_rotation_ = p.as_double();}},
      {"k_avel_rotation", [this](const rclcpp::Parameter& p) {k_avel_rotation_ = p.as_double();}},
      {"goal_tolerance_lin_vel", [this](const rclcpp::Parameter& p) {goal_tolerance_lin_vel_ = p.as_double();}},
      {"goal_tolerance_ang_vel", [this](const rclcpp::Parameter& p) {goal_tolerance_ang_vel_ = p.as_double();}},
    };

  for (const auto& [key, callback]: on_parameter_map)
  {
    callback_handles_.push_back(parameter_hander_->add_parameter_callback(key, callback));
  }

}

TrackerNode::~TrackerNode()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub_vel_->publish(cmd_vel);
}

void TrackerNode::cbSpeed(const std_msgs::msg::Float32::ConstPtr& msg)
{
  vel_[0] = msg->data;
}

template <typename MSG_TYPE>
void TrackerNode::cbPath(const typename MSG_TYPE::ConstPtr& msg)
{
  path_header_ = msg->header;
  is_path_updated_ = true;
  path_step_done_ = 0;
  path_.fromMsg(*msg, epsilon_);
  for (const auto& path_pose : path_)
  {
    if (std::isfinite(path_pose.velocity_) && path_pose.velocity_ < -0.0)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "path_velocity.velocity.x must be positive");
      path_.clear();
      return;
    }
  }
}

void TrackerNode::cbOdometry(const nav_msgs::msg::Odometry::ConstPtr& odom)
{
  if (odom->header.frame_id != frame_odom_)
  {
    RCLCPP_WARN(this->get_logger(), "frame_odom is invalid. Update from \"%s\" to \"%s\"", frame_odom_.c_str(), odom->header.frame_id.c_str());
    frame_odom_ = odom->header.frame_id;
  }
  if (odom->child_frame_id != frame_robot_)
  {
    RCLCPP_WARN(this->get_logger(), "frame_robot is invalid. Update from \"%s\" to \"%s\"",
             frame_robot_.c_str(), odom->child_frame_id.c_str());
    frame_robot_ = odom->child_frame_id;
  }
  if (odom_timeout_sec_ != 0.0)
  {
    {
      odom_timeout_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(odom_timeout_sec_),
        std::bind(&TrackerNode::cbOdomTimeout, this));
    }
  }

  if (prev_odom_stamp_ != rclcpp::Time(0LL, prev_odom_stamp_.get_clock_type()))
  {
    const double dt = std::min(max_dt_, (rclcpp::Time(odom->header.stamp) - prev_odom_stamp_).seconds());
    nav_msgs::msg::Odometry odom_compensated = *odom;
    Eigen::Vector3d prediction_offset(0, 0, 0);
    if (predict_odom_)
    {
      const double predict_dt = std::max(0.0, std::min(max_dt_, (now() - odom->header.stamp).seconds()));
      tf2::Transform trans;
      const tf2::Quaternion rotation(tf2::Vector3(0, 0, 1), odom->twist.twist.angular.z * predict_dt);
      const tf2::Vector3 translation(odom->twist.twist.linear.x * predict_dt, 0, 0);

      prediction_offset[0] = odom->twist.twist.linear.x * predict_dt;
      prediction_offset[2] = odom->twist.twist.angular.z * predict_dt;

      tf2::fromMsg(odom->pose.pose, trans);
      trans.setOrigin(trans.getOrigin() + tf2::Transform(trans.getRotation()) * translation);
      trans.setRotation(trans.getRotation() * rotation);
      tf2::toMsg(trans, odom_compensated.pose.pose);
    }

    tf2::Transform odom_to_robot;
    tf2::fromMsg(odom_compensated.pose.pose, odom_to_robot);
    const tf2::Stamped<tf2::Transform> odom_to_robot_stamped(odom_to_robot, tf2_ros::fromRclcpp(odom->header.stamp), odom->header.frame_id);
    control(odom_to_robot_stamped, prediction_offset, odom->twist.twist.linear.x, odom->twist.twist.angular.z, dt);
  }
  prev_odom_stamp_ = odom->header.stamp;
}

void TrackerNode::cbTimer()
{
  try
  {
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(
        tfbuf_->lookupTransform(frame_odom_, frame_robot_, rclcpp::Time(0)), transform);
    control(transform, Eigen::Vector3d(0, 0, 0), 0, 0, 1.0 / hz_);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF exception: %s", e.what());
    trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
    status.header.stamp = now();
    status.distance_remains = 0.0;
    status.angle_remains = 0.0;
    status.path_header = path_header_;
    status.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
    pub_status_->publish(status);
    return;
  }
}

void TrackerNode::cbOdomTimeout()
{
  RCLCPP_WARN_STREAM(this->get_logger(), "Odometry timeout. Last odometry stamp: " << prev_odom_stamp_.seconds());
  v_lim_.clear();
  w_lim_.clear();
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  pub_vel_->publish(cmd_vel);

  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
  status.header.stamp = now();
  status.distance_remains = 0.0;
  status.angle_remains = 0.0;
  status.path_header = path_header_;
  status.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
  pub_status_->publish(status);
  odom_timeout_timer_->cancel();
}

void TrackerNode::spin()
{
  rclcpp::TimerBase::SharedPtr timer;
  if (!use_odom_)
  {
    timer = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz_),
      std::bind(&TrackerNode::cbTimer, this));
  }
  rclcpp::spin(shared_from_this());
}

void TrackerNode::control(
    const tf2::Stamped<tf2::Transform>& odom_to_robot,
    const Eigen::Vector3d& prediction_offset,
    const double odom_linear_vel,
    const double odom_angular_vel,
    const double dt)
{
  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
  status.header.stamp = now();
  status.path_header = path_header_;
  if (is_path_updated_)
  {
    // Call getTrackingResult to update path_step_done_.
    const TrackingResult initial_tracking_result =
        getTrackingResult(odom_to_robot, prediction_offset, odom_linear_vel, odom_angular_vel);
    path_step_done_ = initial_tracking_result.path_step_done;
    is_path_updated_ = false;
  }
  const TrackingResult tracking_result =
      getTrackingResult(odom_to_robot, prediction_offset, odom_linear_vel, odom_angular_vel);
  switch (tracking_result.status)
  {
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH:
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH:
    {
      v_lim_.clear();
      w_lim_.clear();
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      pub_vel_->publish(cmd_vel);
      break;
    }
    default:
    {
      if (tracking_result.turning_in_place)
      {
        v_lim_.set(0.0, tracking_result.target_linear_vel, acc_[0], dt);

        if (use_time_optimal_control_)
        {
          const double expected_angle_remains =
              tracking_result.angle_remains + w_lim_.get() * dt * time_optimal_control_future_gain_;
          w_lim_.set(trajectory_tracker::timeOptimalControl(expected_angle_remains, acc_toc_[1]), vel_[1], acc_[1], dt);
        }
        else
        {
          const double wvel_increment =
              (-tracking_result.angle_remains * k_ang_rotation_ - w_lim_.get() * k_avel_rotation_) * dt;
          w_lim_.increment(wvel_increment, vel_[1], acc_[1], dt);
        }
        RCLCPP_DEBUG(
            this->get_logger(),
            "trajectory_tracker: angular residual %0.3f, angular vel %0.3f",
            tracking_result.angle_remains, w_lim_.get());
      }
      else
      {
        v_lim_.set(
            trajectory_tracker::timeOptimalControl(tracking_result.signed_local_distance, acc_toc_[0]),
            tracking_result.target_linear_vel, acc_[0], dt);

        float wref = std::abs(v_lim_.get()) * tracking_result.tracking_point_curv;

        if (limit_vel_by_avel_ && std::abs(wref) > vel_[1])
        {
          v_lim_.set(
              std::copysign(1.0, v_lim_.get()) * std::abs(vel_[1] / tracking_result.tracking_point_curv),
              tracking_result.target_linear_vel, acc_[0], dt);
          wref = std::copysign(1.0, wref) * vel_[1];
        }

        const double k_ang =
            (gain_at_vel_ == 0.0) ? (k_[1]) : (k_[1] * tracking_result.target_linear_vel / gain_at_vel_);
        const double dist_diff = tracking_result.distance_from_target;
        const double angle_diff = tracking_result.angle_remains;
        const double wvel_diff = w_lim_.get() - wref;
        w_lim_.increment(dt * (-dist_diff * k_[0] - angle_diff * k_ang - wvel_diff * k_[2]), vel_[1], acc_[1], dt);

        RCLCPP_DEBUG(
            this->get_logger(),
            "trajectory_tracker: distance residual %0.3f, angular residual %0.3f, ang vel residual %0.3f"
            ", v_lim %0.3f, w_lim %0.3f signed_local_distance %0.3f, k_ang %0.3f",
            dist_diff, angle_diff, wvel_diff, v_lim_.get(), w_lim_.get(), tracking_result.signed_local_distance, k_ang);
      }
      if (std::abs(tracking_result.distance_remains) < stop_tolerance_dist_ &&
          std::abs(tracking_result.angle_remains) < stop_tolerance_ang_ &&
          std::abs(tracking_result.distance_remains_raw) < stop_tolerance_dist_ &&
          std::abs(tracking_result.angle_remains_raw) < stop_tolerance_ang_)
      {
        v_lim_.clear();
        w_lim_.clear();
      }
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = v_lim_.get();
      cmd_vel.angular.z = w_lim_.get();
      pub_vel_->publish(cmd_vel);
      path_step_done_ = tracking_result.path_step_done;
      break;
    }
  }
  status.status = tracking_result.status;
  status.distance_remains = tracking_result.distance_remains;
  status.angle_remains = tracking_result.angle_remains;
  pub_status_->publish(status);
}

TrackerNode::TrackingResult TrackerNode::getTrackingResult(
    const tf2::Stamped<tf2::Transform>& odom_to_robot, const Eigen::Vector3d& prediction_offset,
    const double odom_linear_vel, const double odom_angular_vel)
{
  if (path_header_.frame_id.size() == 0 || path_.size() == 0)
  {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }
  // Transform
  trajectory_tracker::Path2D lpath;
  double transform_delay = 0;
  try
  {
    tf2::Stamped<tf2::Transform> path_to_odom;
    tf2::fromMsg(
        tfbuf_->lookupTransform(path_header_.frame_id, frame_odom_, rclcpp::Time(0)), path_to_odom);
    const tf2::Transform path_to_robot = path_to_odom * odom_to_robot;
    transform_delay = (now() - rclcpp::Time(tf2_ros::toRclcpp(path_to_odom.stamp_), get_clock()->get_clock_type())).seconds();
    if (std::abs(transform_delay) > 0.1 && check_old_path_)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(),
          1000, "Timestamp of the transform is too old %f %f",
          now().seconds(), tf2_ros::toRclcpp(path_to_odom.stamp_).seconds());
    }
    const float robot_yaw = tf2::getYaw(path_to_robot.getRotation());
    const Eigen::Transform<double, 2, Eigen::TransformTraits::AffineCompact> path_to_robot_2d =
        Eigen::Translation2d(
            Eigen::Vector2d(path_to_robot.getOrigin().x(), path_to_robot.getOrigin().y())) *
        Eigen::Rotation2Dd(robot_yaw);
    const auto robot_to_path_2d = path_to_robot_2d.inverse();

    for (size_t i = 0; i < path_.size(); i += path_step_)
      lpath.push_back(
          trajectory_tracker::Pose2D(
              robot_to_path_2d * path_[i].pos_, -robot_yaw + path_[i].yaw_, path_[i].velocity_));
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "TF exception: %s", e.what());
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const Eigen::Vector2d origin_raw = prediction_offset.head<2>();
  const float yaw_raw = prediction_offset[2];

  const float yaw_predicted = w_lim_.get() * look_forward_ / 2;
  const Eigen::Vector2d origin =
      Eigen::Vector2d(std::cos(yaw_predicted), std::sin(yaw_predicted)) * v_lim_.get() * look_forward_;

  const double path_length = lpath.length();

  // Find nearest line strip
  const trajectory_tracker::Path2D::ConstIterator it_local_goal =
      lpath.findLocalGoal(lpath.cbegin() + path_step_done_, lpath.cend(), allow_backward_);

  const float max_search_range = (path_step_done_ > 0) ? 1.0 : 0.0;
  const trajectory_tracker::Path2D::ConstIterator it_nearest =
      lpath.findNearest(lpath.cbegin() + path_step_done_, it_local_goal, origin,
                        max_search_range, epsilon_);

  if (it_nearest == lpath.end())
  {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const int i_nearest = std::distance(lpath.cbegin(), it_nearest);
  const int i_nearest_prev = std::max(0, i_nearest - 1);
  const int i_local_goal = std::distance(lpath.cbegin(), it_local_goal);

  const Eigen::Vector2d pos_on_line =
      trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);
  const Eigen::Vector2d pos_on_line_raw =
      trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin_raw);

  const float linear_vel =
      std::isnan(lpath[i_nearest].velocity_) ? vel_[0] : lpath[i_nearest].velocity_;

  // Remained distance to the local goal
  float remain_local = lpath.remainedDistance(lpath.cbegin(), it_nearest, it_local_goal, pos_on_line);
  // Remained distance to the final goal
  float distance_remains = lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line);
  float distance_remains_raw = lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line_raw);
  if (path_length < no_pos_cntl_dist_)
    distance_remains = distance_remains_raw = remain_local = 0;

  // Signed distance error
  const float dist_err = trajectory_tracker::lineDistance(
      lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);

  // Angular error
  const Eigen::Vector2d vec = lpath[i_nearest].pos_ - lpath[i_nearest_prev].pos_;
  float angle_remains = -atan2(vec[1], vec[0]);
  const float angle_pose = allow_backward_ ? lpath[i_nearest].yaw_ : -angle_remains;
  float sign_vel = 1.0;
  if (std::cos(-angle_remains) * std::cos(angle_pose) + std::sin(-angle_remains) * std::sin(angle_pose) < 0)
  {
    sign_vel = -1.0;
    angle_remains = angle_remains + M_PI;
  }
  angle_remains = trajectory_tracker::angleNormalized(angle_remains);

  // Curvature
  const float curv = lpath.getCurvature(it_nearest, it_local_goal, pos_on_line, curv_forward_);

  RCLCPP_DEBUG(
      this->get_logger(),
      "trajectory_tracker: nearest: %d, local goal: %d, done: %d, goal: %lu, remain: %0.3f, remain_local: %0.3f",
      i_nearest, i_local_goal, path_step_done_, lpath.size(), distance_remains, remain_local);

  bool arrive_local_goal(false);
  bool in_place_turning = (vec[1] == 0.0 && vec[0] == 0.0);

  TrackingResult result(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING);

  // Stop and rotate
  const bool large_angle_error = std::abs(rotate_ang_) < M_PI && std::cos(rotate_ang_) > std::cos(angle_remains);
  if (large_angle_error ||
      std::abs(remain_local) < stop_tolerance_dist_ ||
      path_length < min_track_path_ ||
      in_place_turning)
  {
    if (large_angle_error)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Stop and rotate due to large angular error: %0.3f", angle_remains);
    }

    if (path_length < min_track_path_ ||
        std::abs(remain_local) < stop_tolerance_dist_ ||
        in_place_turning)
    {
      angle_remains = trajectory_tracker::angleNormalized(-(it_local_goal - 1)->yaw_);
      if (it_local_goal != lpath.end())
        arrive_local_goal = true;
    }
    if (path_length < stop_tolerance_dist_ || in_place_turning)
      distance_remains = distance_remains_raw = 0.0;

    result.turning_in_place = true;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
  }
  else
  {
    // Too far from given path
    float dist_from_path = dist_err;
    if (i_nearest == 0)
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    else if (i_nearest + 1 >= static_cast<int>(path_.size()))
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    if (std::abs(dist_from_path) > d_stop_)
    {
      result.distance_remains = distance_remains;
      result.distance_remains_raw = distance_remains_raw;
      result.angle_remains = angle_remains;
      result.angle_remains_raw = angle_remains + yaw_raw;
      result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH;
      return result;
    }

    // Path following control
    result.turning_in_place = false;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
    result.angle_remains_raw = angle_remains + yaw_raw;
    result.distance_from_target = trajectory_tracker::clip(dist_err, d_lim_);
    result.signed_local_distance = -remain_local * sign_vel;
    result.tracking_point_curv = curv;
    result.tracking_point_x = pos_on_line[0];
    result.tracking_point_y = pos_on_line[1];
  }

  if (std::abs(result.distance_remains) < goal_tolerance_dist_ &&
      std::abs(result.angle_remains) < goal_tolerance_ang_ &&
      std::abs(result.distance_remains_raw) < goal_tolerance_dist_ &&
      std::abs(result.angle_remains_raw) < goal_tolerance_ang_ &&
      (goal_tolerance_lin_vel_ == 0.0 || std::abs(odom_linear_vel) < goal_tolerance_lin_vel_) &&
      (goal_tolerance_ang_vel_ == 0.0 || std::abs(odom_angular_vel) < goal_tolerance_ang_vel_) &&
      it_local_goal == lpath.end())
  {
    result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL;
  }

  if (arrive_local_goal)
    result.path_step_done = i_local_goal;
  else
    result.path_step_done = std::max(path_step_done_, i_nearest - 1);

  return result;
}
}  // namespace trajectory_tracker

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto track = std::make_shared<trajectory_tracker::TrackerNode>();
  track->spin();

  return 0;
}
