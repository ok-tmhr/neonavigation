/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


class DummyRobotNode : public rclcpp::Node
{
protected:
  double x_;
  double y_;
  double yaw_;
  float v_;
  float w_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  void cbTwist(const geometry_msgs::msg::Twist::ConstPtr& msg)
  {
    v_ = msg->linear.x;
    w_ = msg->angular.z;
  }
  void cbInit(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;
    try
    {
      geometry_msgs::msg::TransformStamped trans =
          tfbuf_->lookupTransform("odom", pose_in.header.frame_id, pose_in.header.stamp, rclcpp::Duration(1, 0));
      tf2::doTransform(pose_in, pose_out, trans);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      return;
    }

    x_ = pose_out.pose.position.x;
    y_ = pose_out.pose.position.y;
    yaw_ = tf2::getYaw(pose_out.pose.orientation);
    v_ = 0;
    w_ = 0;
  }

public:
  DummyRobotNode()
    : rclcpp::Node("dummy_robot")
  {
    x_ = this->declare_parameter("initial_x", 0.0);
    y_ = this->declare_parameter("initial_y", 0.0);
    yaw_ = this->declare_parameter("initial_yaw", 0.0);
    v_ = 0.0;
    w_ = 0.0;

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1).transient_local());
    using std::placeholders::_1;
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&DummyRobotNode::cbTwist, this, _1));
    sub_init_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&DummyRobotNode::cbInit, this, _1));

    tfbuf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
  }
  void spin()
  {
    const float dt = 0.01;
    rclcpp::Rate rate(1.0 / dt);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
      const rclcpp::Time current_time = now();

      yaw_ += w_ * dt;
      x_ += cosf(yaw_) * v_ * dt;
      y_ += sinf(yaw_) * v_ * dt;

      geometry_msgs::msg::TransformStamped trans;
      trans.header.stamp = current_time;
      trans.header.frame_id = "odom";
      trans.child_frame_id = "base_link";
      trans.transform.translation = tf2::toMsg(tf2::Vector3(x_, y_, 0.0));
      trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
      tfb_->sendTransform(trans);

      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "odom";
      odom.header.stamp = current_time;
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
      odom.twist.twist.linear.x = v_;
      odom.twist.twist.angular.z = w_;
      pub_odom_->publish(odom);
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto robot = std::make_shared<DummyRobotNode>();
  robot->spin();

  return 0;
}
