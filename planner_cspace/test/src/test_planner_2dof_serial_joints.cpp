/*
 * Copyright (c) 2017, the neonavigation authors
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
#include <planner_cspace_msgs/msg/planner_status.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <gtest/gtest.h>

TEST(Planner2DOFSerialJoints, Plan)
{
  auto nh = rclcpp::Node::make_shared("test_planner_2dof_serial_joints");
  auto pub_state = nh->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(1).transient_local());
  auto pub_cmd = nh->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_in", rclcpp::QoS(1).transient_local());

  trajectory_msgs::msg::JointTrajectory::ConstPtr planned;
  const auto cb_plan = [&planned](const trajectory_msgs::msg::JointTrajectory::ConstPtr& msg)
  {
    planned = msg;
  };
  auto sub_plan = nh->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 1, cb_plan);

  planner_cspace_msgs::msg::PlannerStatus::ConstPtr status;
  const auto cb_status = [&status](const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg)
  {
    status = msg;
  };
  auto sub_status = nh->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_2dof_serial_joints/group0/status", 1, cb_status);

  sensor_msgs::msg::JointState s;
  s.name.push_back("front");
  s.position.push_back(-1.57);
  s.name.push_back("rear");
  s.position.push_back(0.0);

  trajectory_msgs::msg::JointTrajectory cmd;
  cmd.joint_names.push_back("front");
  cmd.joint_names.push_back("rear");
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.push_back(-4.71);
  p.positions.push_back(0.0);
  cmd.points.push_back(p);

  rclcpp::Rate rate(1);
  const rclcpp::Time deadline = nh->now() + rclcpp::Duration::from_seconds(10);
  int cnt = 0;
  while (rclcpp::ok())
  {
    if (nh->now() > deadline)
    {
      FAIL() << "Timeout";
    }

    pub_state->publish(s);
    pub_cmd->publish(cmd);

    rclcpp::spin_some(nh);
    rate.sleep();
    if (planned && status)
    {
      cnt++;
      if (cnt > 5)
      {
        break;
      }
    }
  }
  ASSERT_TRUE(rclcpp::ok());

  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DOING, status->status);
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL, status->error);

  ASSERT_EQ(2u, planned->joint_names.size());
  ASSERT_EQ("front", planned->joint_names[0]);
  ASSERT_EQ("rear", planned->joint_names[1]);

  // collision at front=-3.14, rear=0.0 must be avoided.
  const float fc = -3.14;
  const float rc = 0.0;
  for (int i = 1; i < static_cast<int>(planned->points.size()); ++i)
  {
    const trajectory_msgs::msg::JointTrajectoryPoint& p0 = planned->points[i - 1];
    const trajectory_msgs::msg::JointTrajectoryPoint& p1 = planned->points[i];
    ASSERT_EQ(2u, p0.positions.size());
    ASSERT_EQ(2u, p1.positions.size());

    const float f0 = p0.positions[0];
    const float r0 = p0.positions[1];
    const float f1 = p1.positions[0];
    const float r1 = p1.positions[1];

    ASSERT_LT(-6.28, f0);
    ASSERT_GT(0.0, f0);
    ASSERT_LT(-3.14, r0);
    ASSERT_GT(3.14, r0);
    ASSERT_LT(-6.28, f1);
    ASSERT_GT(0.0, f1);
    ASSERT_LT(-3.14, r1);
    ASSERT_GT(3.14, r1);

    const float front_diff = f1 - f0;
    const float rear_diff = r1 - r0;

    const float d =
        std::abs(
            rear_diff * fc -
            front_diff * rc +
            f1 * r0 -
            r1 * f0) /
        std::hypot(front_diff, rear_diff);
    // std::cerr << d << std::endl;
    ASSERT_GT(d, 0.15);

    ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DOING, status->status);
    ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL, status->error);
  }
}

TEST(Planner2DOFSerialJoints, NoPath)
{
  auto nh = rclcpp::Node::make_shared("test_planner_2dof_serial_joints");
  auto pub_state = nh->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(1).transient_local());
  auto pub_cmd = nh->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_in", rclcpp::QoS(1).transient_local());

  planner_cspace_msgs::msg::PlannerStatus::ConstPtr status;
  const auto cb_status = [&status](const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg)
  {
    status = msg;
  };
  auto sub_status = nh->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_2dof_serial_joints/group0/status", 1, cb_status);

  sensor_msgs::msg::JointState s;
  s.name.push_back("front");
  s.position.push_back(-1.57);
  s.name.push_back("rear");
  s.position.push_back(0.0);

  trajectory_msgs::msg::JointTrajectory cmd;
  cmd.joint_names.push_back("front");
  cmd.joint_names.push_back("rear");
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.push_back(3.14);
  p.positions.push_back(0.0);  // Collided state
  cmd.points.push_back(p);

  rclcpp::Rate rate(1);
  const rclcpp::Time deadline = nh->now() + rclcpp::Duration::from_seconds(10);
  int cnt = 0;
  while (rclcpp::ok())
  {
    if (nh->now() > deadline)
    {
      FAIL() << "Timeout";
    }

    pub_state->publish(s);
    pub_cmd->publish(cmd);

    rclcpp::spin_some(nh);
    rate.sleep();
    if (status)
    {
      cnt++;
      if (cnt > 5)
      {
        break;
      }
    }
  }
  ASSERT_TRUE(rclcpp::ok());

  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DOING, status->status);
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND, status->error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
