/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <cstddef>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <planner_cspace_msgs/msg/planner_status.hpp>
#include <std_msgs/msg/empty.hpp>

#include <gtest/gtest.h>

class NavigateBoundary : public ::testing::Test
{
protected:
  using ActionClient = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;

  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Subscription<>::SharedPtr sub_status_;
  rclcpp::Subscription<>::SharedPtr sub_path_;
  planner_cspace_msgs::msg::PlannerStatus::ConstPtr status_;
  nav_msgs::msg::Path::ConstPtr path_;
  ActionClientPtr move_base_;

  NavigateBoundary()
  {
    move_base_ = std::make_shared<ActionClient>("/move_base");
    if (!move_base_->waitForServer(rclcpp::Duration::from_seconds(10.0)))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect move_base action");
      exit(EXIT_FAILURE);
    }
  }

  void publishTransform(const double x, const double y)
  {
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = this->now();
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = x;
    trans.transform.translation.y = y;
    trans.transform.rotation.w = 1.0;
    tfb_.sendTransform(trans);
  }
  virtual void SetUp()
  {
    sub_status_ = nh_->create_subscription("/planner_3d/status", 100, &NavigateBoundary::cbStatus, this);
    sub_path_ = nh_->create_subscription("path", 1, &NavigateBoundary::cbPath, this);

    publishTransform(1.0, 0.6);
    rclcpp::Duration::from_seconds(0.5).sleep();

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = this->now();
    goal.target_pose.pose.orientation.w = 1;
    goal.target_pose.pose.position.x = 1.4;
    goal.target_pose.pose.position.y = 0.6;
    move_base_->async_send_goal(goal);
    rclcpp::Duration::from_seconds(0.5).sleep();
  }
  void cbPath(const nav_msgs::msg::Path::ConstPtr& msg)
  {
    path_ = msg;
  }
  void cbStatus(const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg)
  {
    status_ = msg;
  }
};

TEST_F(NavigateBoundary, StartPositionScan)
{
  // map width/height is 32px * 0.1m = 3.2m
  for (double x = -10; x < 13; x += 2.0)
  {
    for (double y = -10; y < 13; y += 2.0)
    {
      publishTransform(x, y);

      path_ = nullptr;
      status_ = nullptr;
      for (int i = 0; i < 100; ++i)
      {
        rclcpp::Duration::from_seconds(0.05).sleep();
        rclcpp::spin_some(shared_from_this());
        if (path_ && status_)
          break;
      }
      // Planner must publish at least empty path if alive.
      ASSERT_TRUE(static_cast<bool>(path_));
      // Planner status must be published even if robot if outside of the map.
      ASSERT_TRUE(static_cast<bool>(status_));
    }
  }
}

TEST_F(NavigateBoundary, StartPositionScanWithTemporaryEscape)
{
  rclcpp::Publisher<>::SharedPtr pub_trigger = nh_->create_publisher<std_msgs::msg::Empty>("/planner_3d/temporary_escape", 1);

  // map width/height is 32px * 0.1m = 3.2m
  for (double x = -10; x < 13; x += 2.0)
  {
    for (double y = -10; y < 13; y += 2.0)
    {
      publishTransform(x, y);

      path_ = nullptr;
      status_ = nullptr;
      for (int i = 0; i < 10; ++i)
      {
        std_msgs::msg::Empty msg;
        pub_trigger->publish(msg);

        rclcpp::Duration::from_seconds(0.2).sleep();
        rclcpp::spin_some(shared_from_this());
        if (path_ && status_)
          break;
      }
      // Planner must publish at least empty path if alive.
      ASSERT_TRUE(static_cast<bool>(path_));
      // Planner status must be published even if robot if outside of the map.
      ASSERT_TRUE(static_cast<bool>(status_));
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv, "test_navigate_boundary");

  return RUN_ALL_TESTS();
}
