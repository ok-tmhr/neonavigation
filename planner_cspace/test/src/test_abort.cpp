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

class AbortTest
  : public ActionTestBase<nav2_msgs::action::NavigateToPose, ACTION_TOPIC_MOVE_BASE>
{
protected:
  nav2_msgs::action::NavigateToPose_Goal createGoalInRock()
  {
    nav2_msgs::action::NavigateToPose_Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 1.19;
    goal.pose.pose.position.y = 1.90;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    return goal;
  }
  nav2_msgs::action::NavigateToPose_Goal createGoalInFree()
  {
    nav2_msgs::action::NavigateToPose_Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 2.1;
    goal.pose.pose.position.y = 0.45;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 1.0;
    goal.pose.pose.orientation.w = 0.0;
    return goal;
  }
};

TEST_F(AbortTest, AbortByGoalInRock)
{
  const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(10);
  rclcpp::Rate wait(1.0);

  // Assure that goal is received after map in planner_3d.
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  // Send a goal which is in Rock
  auto future = move_base_->async_send_goal(createGoalInRock());
  while (future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING)
  {
    wait.sleep();
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't get active: " << future.get()->get_status()
        << " " << statusString();
  }

  // Try to replan
  while (future.get()->get_status() ==
         rclcpp_action::GoalStatus::STATUS_EXECUTING)
  {
    wait.sleep();
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't get inactive: " << future.get()->get_status()
        << " " << statusString();
  }
  wait.sleep();

  ASSERT_TRUE(planner_status_);

  // Abort after exceeding max_retry_num
  ASSERT_EQ(rclcpp_action::GoalStatus::STATUS_ABORTED,
            future.get()->get_status());
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND,
            planner_status_->error);

  // Send another goal which is not in Rock
  future = move_base_->async_send_goal(createGoalInFree());
  while (future.get()->get_status() !=
         rclcpp_action::GoalStatus::STATUS_EXECUTING)
  {
    wait.sleep();
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't get active: " << future.get()->get_status()
        << " " << statusString();
  }
  while (future.get()->get_status() ==
         rclcpp_action::GoalStatus::STATUS_EXECUTING)
  {
    wait.sleep();
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't get inactive: " << future.get()->get_status()
        << " " << statusString();
  }
  wait.sleep();

  // Succeed
  ASSERT_EQ(rclcpp_action::GoalStatus::STATUS_SUCCEEDED,
            future.get()->get_status());
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL,
            planner_status_->error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  return ret;
}
