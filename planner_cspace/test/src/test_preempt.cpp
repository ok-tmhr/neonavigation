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

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <planner_cspace_msgs/msg/planner_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <planner_cspace/action_test_base.h>

class PreemptTest
  : public ActionTestBase<nav2_msgs::action::NavigateToPose, ACTION_TOPIC_MOVE_BASE>
{
protected:
  nav2_msgs::action::NavigateToPose_Goal CreateGoalInFree()
  {
    nav2_msgs::action::NavigateToPose_Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 1.24;
    goal.pose.pose.position.y = 0.65;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    return goal;
  }
};

TEST_F(PreemptTest, Preempt)
{
  const rclcpp::Time deadline = node_->now() + rclcpp::Duration(5, 0);
  rclcpp::Rate wait(1.0);

  auto future = move_base_->async_send_goal(CreateGoalInFree());

  while (true)
  {
    rclcpp::spin_some(node_);
    wait.sleep();
    if (future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
    {
      break;
    }
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't get active: " << future.get()->get_status()
        << statusString();
  }
  while (true)
  {
    rclcpp::spin_some(node_);
    move_base_->async_cancel_all_goals();
    wait.sleep();
    if (future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING)
    {
      break;
    }
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't get inactive: " << future.get()->get_status()
        << statusString();
  }

  ASSERT_TRUE(planner_status_);

  ASSERT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED,
            future.get()->get_status());
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL,
            planner_status_->error);
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DONE,
            planner_status_->status);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  return ret;
}
