/*
 * Copyright (c) 2018-2019, the neonavigation authors
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

#ifndef TEST_SAFETY_LIMITER_BASE_H
#define TEST_SAFETY_LIMITER_BASE_H

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <safety_limiter_msgs/msg/safety_limiter_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtest/gtest.h>

namespace
{
inline void GenerateEmptyPointcloud2(sensor_msgs::msg::PointCloud2& cloud)
{
  cloud.height = 1;
  cloud.width = 0;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
}
inline void GenerateSinglePointPointcloud2(
    sensor_msgs::msg::PointCloud2& cloud,
    const float x,
    const float y,
    const float z)
{
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  modifier.resize(1);
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
}
}  // namespace

class SafetyLimiterTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_watchdog_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diag_;
  rclcpp::Subscription<safety_limiter_msgs::msg::SafetyLimiterStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;

  inline void cbDiag(const diagnostic_msgs::msg::DiagnosticArray::ConstPtr& msg)
  {
    diag_ = msg;
  }

  inline void cbStatus(const safety_limiter_msgs::msg::SafetyLimiterStatus::ConstPtr& msg)
  {
    status_ = msg;
  }

  inline void cbCmdVel(const geometry_msgs::msg::Twist::ConstPtr& msg)
  {
    cmd_vel_ = msg;
  }

public:
  diagnostic_msgs::msg::DiagnosticArray::ConstPtr diag_;
  safety_limiter_msgs::msg::SafetyLimiterStatus::ConstPtr status_;
  geometry_msgs::msg::Twist::ConstPtr cmd_vel_;

  inline SafetyLimiterTest()
  {
    nh_ = rclcpp::Node::make_shared("test_safety_limiter");
    tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
    pub_cmd_vel_ = nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_in", 1);
    pub_cloud_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
    pub_watchdog_ = nh_->create_publisher<std_msgs::msg::Empty>("watchdog_reset", 1);
    using std::placeholders::_1;
    sub_diag_ = nh_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1, std::bind(&SafetyLimiterTest::cbDiag, this, _1));
    sub_status_ = nh_->create_subscription<safety_limiter_msgs::msg::SafetyLimiterStatus>("/safety_limiter/status", rclcpp::QoS(1).transient_local(), std::bind(&SafetyLimiterTest::cbStatus, this, _1));
    sub_cmd_vel_ = nh_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1).transient_local(), std::bind(&SafetyLimiterTest::cbCmdVel, this, _1));

    rclcpp::Rate wait(10.0);
    // Skip initial state
    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      publishEmptyPointPointcloud2("base_link", nh_->now());
      publishWatchdogReset();
      broadcastTF("odom", "base_link", 0.0, 0.0);

      wait.sleep();
      rclcpp::spin_some(nh_);
    }
    cmd_vel_.reset();
    diag_.reset();
    status_.reset();
  }
  inline void publishWatchdogReset()
  {
    std_msgs::msg::Empty watchdog_reset;
    pub_watchdog_->publish(watchdog_reset);
  }
  inline void publishEmptyPointPointcloud2(
      const std::string frame_id,
      const rclcpp::Time stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
    GenerateEmptyPointcloud2(cloud);
    pub_cloud_->publish(cloud);
  }
  inline void publishSinglePointPointcloud2(
      const float x,
      const float y,
      const float z,
      const std::string frame_id,
      const rclcpp::Time stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
    GenerateSinglePointPointcloud2(cloud, x, y, z);
    pub_cloud_->publish(cloud);
  }
  inline void publishTwist(
      const float lin,
      const float ang,
      const float lin_y = 0.0)
  {
    geometry_msgs::msg::Twist cmd_vel_out;
    cmd_vel_out.linear.x = lin;
    cmd_vel_out.linear.y = lin_y;
    cmd_vel_out.angular.z = ang;
    pub_cmd_vel_->publish(cmd_vel_out);
  }
  inline void broadcastTF(
      const std::string parent_frame_id,
      const std::string child_frame_id,
      const float lin,
      const float ang)
  {
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = nh_->now();
    trans.transform = tf2::toMsg(
        tf2::Transform(tf2::Quaternion(tf2::Vector3(0, 0, 1), ang), tf2::Vector3(lin, 0, 0)));
    trans.header.frame_id = parent_frame_id;
    trans.child_frame_id = child_frame_id;
    tfb_->sendTransform(trans);
  }
  inline bool hasDiag() const
  {
    if (!diag_)
      return false;
    if (diag_->status.size() == 0)
      return false;
    return true;
  }
  inline bool hasStatus() const
  {
    return static_cast<bool>(status_);
  }
};

#endif  // TEST_SAFETY_LIMITER_BASE_H
