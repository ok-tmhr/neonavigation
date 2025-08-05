/*
 * Copyright (c) 2018-2020, the neonavigation authors
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

#ifndef TRAJECTORY_TRACKER_TEST_H
#define TRAJECTORY_TRACKER_TEST_H

#include <algorithm>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>


#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// #include <trajectory_tracker/TrajectoryTrackerConfig.h>
#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>
#include <trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp>

#include <gtest/gtest.h>

class TrajectoryTrackerTest : public ::testing::Test
{
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::PathWithVelocity>::SharedPtr pub_path_vel_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Time cmd_vel_time_;
  rclcpp::Time trans_stamp_last_;

  rclcpp::Time initial_cmd_vel_time_;
  int cmd_vel_count_;

  std::list<nav_msgs::msg::Odometry> odom_buffer_;

protected:
  std_msgs::msg::Header last_path_header_;
  double error_lin_;
  double error_large_lin_;
  double error_ang_;

  double getYaw() const
  {
    return tf2::getYaw(pose_.getRotation());
  }
  Eigen::Vector2d getPos() const
  {
    return Eigen::Vector2d(pose_.getOrigin().getX(), pose_.getOrigin().getY());
  }

private:
  void cbStatus(const trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::ConstPtr& msg)
  {
    status_ = msg;
  }
  void cbCmdVel(const geometry_msgs::msg::Twist::ConstPtr& msg)
  {
    const rclcpp::Time now = node_->now();
    if (cmd_vel_time_ == rclcpp::Time(0LL, RCL_ROS_TIME))
      cmd_vel_time_ = now;
    const float dt = std::min((now - cmd_vel_time_).seconds(), 0.1);
    const tf2::Transform pose_diff(tf2::Quaternion(tf2::Vector3(0, 0, 1), msg->angular.z * dt),
                                   tf2::Vector3(msg->linear.x * dt, 0, 0));
    pose_ *= pose_diff;
    cmd_vel_time_ = now;
    cmd_vel_ = msg;
    ++cmd_vel_count_;
  }

public:
  rclcpp::Node::SharedPtr node_;
  tf2::Transform pose_;
  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::ConstPtr status_;
  geometry_msgs::msg::Twist::ConstPtr cmd_vel_;
  rclcpp::Duration delay_;

  TrajectoryTrackerTest(const std::string& node_name = "test_trajectory_tracker")
  : delay_(rclcpp::Duration(0, 0))
  , trans_stamp_last_(0LL, RCL_ROS_TIME)
  , initial_cmd_vel_time_(0LL, RCL_ROS_TIME)
  , cmd_vel_time_(0LL, RCL_ROS_TIME)
  {
    node_ = rclcpp::Node::make_shared(node_name);
    using std::placeholders::_1;
    sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&TrajectoryTrackerTest::cbCmdVel, this, _1));
    sub_status_ = node_->create_subscription<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>(
      "trajectory_tracker/status", 1, std::bind(&TrajectoryTrackerTest::cbStatus, this, _1));
    pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(1).transient_local());
    pub_path_vel_ = node_->create_publisher<trajectory_tracker_msgs::msg::PathWithVelocity>(
        "path_velocity", rclcpp::QoS(1).transient_local());
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10).transient_local());

    double delay;
    delay = node_->declare_parameter<double>("odom_delay", 0.0);
    delay_ = rclcpp::Duration::from_seconds(delay);
    error_lin_ = node_->declare_parameter<double>("error_lin", 0.01);
    error_large_lin_ = node_->declare_parameter<double>("error_large_lin", 0.1);
    error_ang_ = node_->declare_parameter<double>("error_ang", 0.01);

    tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_.get());

    rclcpp::Rate wait(10);
    for (size_t i = 0; i < 100; ++i)
    {
      wait.sleep();
      rclcpp::spin_some(node_);
      if (pub_path_->get_subscription_count() > 0)
        break;
    }
  }
  void initState(const tf2::Transform& pose)
  {
    // Wait trajectory_tracker node
    rclcpp::Rate rate(10);
    rclcpp::Clock clock;
    const auto start = clock.now();
    while (rclcpp::ok())
    {
      nav_msgs::msg::Path path;
      path.header.frame_id = "odom";
      path.header.stamp = node_->now();
      pub_path_->publish(path);

      pose_ = pose;
      publishTransform();

      rate.sleep();
      rclcpp::spin_some(node_);
      if (status_ &&
          status_->status != trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING)
        break;
      ASSERT_LT(clock.now(), start + rclcpp::Duration::from_seconds(10.0))
          << "trajectory_tracker status timeout, status: "
          << (status_ ? std::to_string(static_cast<int>(status_->status)) : "none");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  void initState(const Eigen::Vector2d& pos, const float yaw)
  {
    initState(tf2::Transform(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw),
                             tf2::Vector3(pos.x(), pos.y(), 0)));
  }
  void waitUntilStart(const std::function<void()> func = nullptr)
  {
    rclcpp::Rate rate(50);
    rclcpp::Clock clock;
    const auto start = clock.now();
    while (rclcpp::ok())
    {
      if (func)
        func();

      publishTransform();
      rate.sleep();
      rclcpp::spin_some(node_);
      if (status_ &&
          status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING)
        break;
      ASSERT_LT(clock.now(), start + rclcpp::Duration::from_seconds(10.0))
          << "trajectory_tracker status timeout, status: "
          << (status_ ? std::to_string(static_cast<int>(status_->status)) : "none");
    }
    initial_cmd_vel_time_ = node_->now();
    cmd_vel_count_ = 0;
  }
  void publishPath(const std::vector<Eigen::Vector3d>& poses)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = node_->now();

    for (const Eigen::Vector3d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      path.poses.push_back(pose);
    }
    pub_path_->publish(path);
    last_path_header_ = path.header;
  }
  void publishPathVelocity(const std::vector<Eigen::Vector4d>& poses)
  {
    trajectory_tracker_msgs::msg::PathWithVelocity path;
    path.header.frame_id = "odom";
    path.header.stamp = node_->now();

    for (const Eigen::Vector4d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      trajectory_tracker_msgs::msg::PoseStampedWithVelocity pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      pose.linear_velocity.x = p[3];

      path.poses.push_back(pose);
    }
    // needs sleep to prevent that the empty path from initState arrives later.
    rclcpp::sleep_for(std::chrono::microseconds(500));
    pub_path_vel_->publish(path);
    last_path_header_ = path.header;
  }
  void publishTransform()
  {
    const rclcpp::Time now = node_->now();

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = now;
    odom.child_frame_id = "base_link";
    tf2::toMsg(pose_, odom.pose.pose);
    if (cmd_vel_)
    {
      odom.twist.twist.linear = cmd_vel_->linear;
      odom.twist.twist.angular = cmd_vel_->angular;
    }
    publishTransform(odom);
  }

  void publishTransform(const nav_msgs::msg::Odometry& odom)
  {
    odom_buffer_.push_back(odom);

    const rclcpp::Time pub_time = rclcpp::Time(odom.header.stamp) - delay_;

    while (odom_buffer_.size() > 0)
    {
      nav_msgs::msg::Odometry odom = odom_buffer_.front();
      if (rclcpp::Time(odom.header.stamp) > pub_time)
        break;

      odom_buffer_.pop_front();

      if (odom.header.stamp != trans_stamp_last_)
      {
        geometry_msgs::msg::TransformStamped trans;
        trans.header = odom.header;
        trans.header.stamp = rclcpp::Time(trans.header.stamp) + rclcpp::Duration::from_seconds(0.1);
        trans.child_frame_id = odom.child_frame_id;
        trans.transform.translation.x = odom.pose.pose.position.x;
        trans.transform.translation.y = odom.pose.pose.position.y;
        trans.transform.rotation.x = odom.pose.pose.orientation.x;
        trans.transform.rotation.y = odom.pose.pose.orientation.y;
        trans.transform.rotation.z = odom.pose.pose.orientation.z;
        trans.transform.rotation.w = odom.pose.pose.orientation.w;

        tfb_->sendTransform(trans);
        pub_odom_->publish(odom);
      }
      trans_stamp_last_ = odom.header.stamp;
    }
  }

  double getCmdVelFrameRate() const
  {
    return cmd_vel_count_ / (cmd_vel_time_ - initial_cmd_vel_time_).seconds();
  }
};

namespace trajectory_tracker_msgs::msg
{
std::ostream& operator<<(std::ostream& os, const TrajectoryTrackerStatus::ConstPtr& msg)
{
  if (!msg)
  {
    os << "nullptr";
  }
  else
  {
    os << "  header: " << rclcpp::Time(msg->header.stamp).seconds() << " " << msg->header.frame_id << std::endl
       << "  distance_remains: " << msg->distance_remains << std::endl
       << "  angle_remains: " << msg->angle_remains << std::endl
       << "  status: " << msg->status << std::endl
       << "  path_header: " << rclcpp::Time(msg->path_header.stamp).seconds() << " " << msg->header.frame_id;
  }
  return os;
}
}  // namespace trajectory_tracker_msgs

#endif  // TRAJECTORY_TRACKER_TEST_H
