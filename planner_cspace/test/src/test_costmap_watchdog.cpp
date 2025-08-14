/*
 * Copyright (c) 2018, the neonavigation authors
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

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <costmap_cspace_msgs/msg/c_space3_d_update.hpp>
#include <planner_cspace_msgs/msg/planner_status.hpp>

#include <gtest/gtest.h>

TEST(Planner3D, CostmapWatchdog)
{
  int cnt = 0;
  planner_cspace_msgs::msg::PlannerStatus::ConstPtr status;
  nav_msgs::msg::Path::ConstPtr path;
  diagnostic_msgs::msg::DiagnosticArray::ConstPtr diag;

  const std::function<void(const planner_cspace_msgs::msg::PlannerStatus::ConstPtr&)> cb_status =
      [&status, &cnt](const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg) -> void
  {
    status = msg;
    cnt++;
  };
  const std::function<void(const nav_msgs::msg::Path::ConstPtr&)> cb_path =
      [&path](const nav_msgs::msg::Path::ConstPtr& msg) -> void
  {
    path = msg;
  };
  const std::function<void(const diagnostic_msgs::msg::DiagnosticArray::ConstPtr&)> cb_diag =
      [&diag](const diagnostic_msgs::msg::DiagnosticArray::ConstPtr& msg) -> void
  {
    diag = msg;
  };

  auto nh = rclcpp::Node::make_shared("test_navigate");
  auto pub_goal = nh->create_publisher<geometry_msgs::msg::PoseStamped>("goal", rclcpp::QoS(1).transient_local());
  auto pub_cost_update = nh->create_publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>("costmap_update", 1);
  auto sub_status = nh->create_subscription<planner_cspace_msgs::msg::PlannerStatus>("planner_3d/status", 1, cb_status);
  auto sub_path = nh->create_subscription<nav_msgs::msg::Path>("path", 1, cb_path);
  auto sub_diag = nh->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1, cb_diag);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.9;
  goal.pose.position.y = 2.8;
  goal.pose.orientation.w = 1.0;
  // Assure that goal is received after map in planner_3d.
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  pub_goal->publish(goal);

  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    // cnt increments in 5 Hz at maximum
    if (cnt == 0 || cnt > 8)
    {
      costmap_cspace_msgs::msg::CSpace3DUpdate update;
      update.header.stamp = nh->now();
      update.header.frame_id = "map";
      update.width = update.height = update.angle = 0;
      pub_cost_update->publish(update);
    }

    rclcpp::spin_some(nh);
    rate.sleep();

    if (!status)
      continue;

    if (5 < cnt && cnt < 8)
    {
      ASSERT_EQ(status->error, planner_cspace_msgs::msg::PlannerStatus::DATA_MISSING);

      ASSERT_TRUE(static_cast<bool>(path));
      ASSERT_EQ(path->poses.size(), 0u);

      ASSERT_TRUE(static_cast<bool>(diag));
      ASSERT_EQ(diag->status.size(), 1u);
      ASSERT_EQ(diag->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      ASSERT_NE(diag->status[0].message.find("missing"), std::string::npos);
    }
    else if (10 < cnt && cnt < 13)
    {
      ASSERT_EQ(status->status, planner_cspace_msgs::msg::PlannerStatus::DOING);
      ASSERT_EQ(status->error, planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);

      ASSERT_EQ(diag->status.size(), 1u);
      ASSERT_EQ(diag->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
      ASSERT_NE(diag->status[0].message.find("well"), std::string::npos);
    }
    else if (cnt >= 13)
    {
      return;
    }
  }
  ASSERT_TRUE(rclcpp::ok());
}

TEST(Planner3D, CostmapTimeoutOnFinishing)
{
  planner_cspace_msgs::msg::PlannerStatus::ConstPtr status;
  nav_msgs::msg::Path::ConstPtr path;

  const std::function<void(const planner_cspace_msgs::msg::PlannerStatus::ConstPtr&)> cb_status =
      [&status](const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg) -> void
  {
    status = msg;
  };
  const std::function<void(const nav_msgs::msg::Path::ConstPtr&)> cb_path =
      [&path](const nav_msgs::msg::Path::ConstPtr& msg) -> void
  {
    path = msg;
  };

  auto nh = rclcpp::Node::make_shared("test_navigate");
  auto pub_goal = nh->create_publisher<geometry_msgs::msg::PoseStamped>("goal", rclcpp::QoS(1).transient_local());
  auto pub_cost_update = nh->create_publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>("costmap_update", 1);
  auto sub_status = nh->create_subscription<planner_cspace_msgs::msg::PlannerStatus>("planner_3d/status", 1, cb_status);
  auto sub_path = nh->create_subscription<nav_msgs::msg::Path>("path", 1, cb_path);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 2.55;
  goal.pose.position.y = 0.45;
  goal.pose.orientation.w = std::sin(0.09);
  goal.pose.orientation.z = std::cos(0.09);
  // Assure that goal is received after map in planner_3d.
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  pub_goal->publish(goal);

  costmap_cspace_msgs::msg::CSpace3DUpdate update;
  update.header.frame_id = "map";
  update.width = update.height = update.angle = 0;

  const rclcpp::Time deadline = nh->now() + rclcpp::Duration::from_seconds(2.0);
  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    update.header.stamp = nh->now();
    pub_cost_update->publish(update);

    rclcpp::spin_some(nh);
    rate.sleep();
    if (status && status->status == planner_cspace_msgs::msg::PlannerStatus::FINISHING)
      break;

    ASSERT_LT(rclcpp::Time(update.header.stamp), deadline)
        << "Planner didn't enter FINISHING state: "
        << (status ? status->status : -1);
  }
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh);
    rate.sleep();
    if (status->error == planner_cspace_msgs::msg::PlannerStatus::DATA_MISSING)
      break;

    ASSERT_EQ(status->status, planner_cspace_msgs::msg::PlannerStatus::FINISHING)
        << "Wrong test condition";
    ASSERT_LT(rclcpp::Time(update.header.stamp), deadline)
        << "Planner didn't enter DATA_MISSING state"
        << status->error;
  }
  path = nullptr;
  while (rclcpp::ok())
  {
    update.header.stamp = nh->now();
    pub_cost_update->publish(update);

    rclcpp::spin_some(nh);
    rate.sleep();
    if (path)
      break;

    ASSERT_EQ(status->status, planner_cspace_msgs::msg::PlannerStatus::FINISHING)
        << "Wrong test condition";
    ASSERT_LT(rclcpp::Time(update.header.stamp), deadline)
        << "No path was published";
  }
  ASSERT_TRUE(rclcpp::ok());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
