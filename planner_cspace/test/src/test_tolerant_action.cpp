/*
 * Copyright (c) 2023, the neonavigation authors
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

#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <planner_cspace_msgs/action/move_with_tolerance.hpp>
#include <planner_cspace_msgs/msg/planner_status.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <planner_cspace/action_test_base.h>

class TolerantActionTest
  : public ActionTestBase<planner_cspace_msgs::action::MoveWithTolerance, ACTION_TOPIC_TOLERANT_MOVE>
{
protected:
  planner_cspace_msgs::action::MoveWithTolerance_Goal createGoalInFree()
  {
    planner_cspace_msgs::action::MoveWithTolerance_Goal goal;
    goal.target_pose.header.stamp = node_->now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 2.1;
    goal.target_pose.pose.position.y = 0.45;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    goal.continuous_movement_mode = true;
    goal.goal_tolerance_ang = 0.1;
    goal.goal_tolerance_ang_finish = 0.05;
    goal.goal_tolerance_lin = 0.2;
    return goal;
  }

  double getDistBetweenRobotAndGoal(const planner_cspace_msgs::action::MoveWithTolerance_Goal& goal)
  {
    try
    {
      const geometry_msgs::msg::TransformStamped map_to_robot =
          tfbuf_.lookupTransform("map", "base_link", rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));
      return std::hypot(map_to_robot.transform.translation.x - goal.target_pose.pose.position.x,
                        map_to_robot.transform.translation.y - goal.target_pose.pose.position.y);
    }
    catch (std::exception&)
    {
      return std::numeric_limits<double>::max();
    }
  }
};

TEST_F(TolerantActionTest, GoalWithTolerance)
{
  const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(10);
  const rclcpp::Duration wait(1, 0);

  // Assure that goal is received after map in planner_3d.
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  const planner_cspace_msgs::action::MoveWithTolerance_Goal goal = createGoalInFree();
  auto future = move_base_->async_send_goal(goal);

  while (rclcpp::ok() && future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING)
  {
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't get active: " << future.get()->get_status()
        << " " << statusString();
    rclcpp::spin_some(node_);
  }

  while (rclcpp::ok() && future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
  {
    ASSERT_LT(node_->now(), deadline)
        << "Action didn't succeeded: " << future.get()->get_status()
        << " " << statusString();
    rclcpp::spin_some(node_);
  }

  const double dist_to_goal = getDistBetweenRobotAndGoal(goal);
  // distance_remains is less than updated goal_tolerance_lin (set in planner_cspace_msgs::action::MoveWithTolerance_Goal).
  EXPECT_LT(dist_to_goal, goal.goal_tolerance_lin);
  // distance_remains is greater than default goal_tolerance_lin (set in actionlib_common_rostest.test).
  EXPECT_GT(dist_to_goal, 0.05);
  // Navigation still continues after ActionClient succeeded.
  EXPECT_EQ(planner_status_->status, planner_cspace_msgs::msg::PlannerStatus::DOING);

  while (rclcpp::ok() && planner_status_->status != planner_cspace_msgs::msg::PlannerStatus::DONE)
  {
    ASSERT_LT(node_->now(), deadline)
        << "Navigation didn't finished: " << future.get()->get_status()
        << " " << statusString();
    rclcpp::spin_some(node_);
  }
  EXPECT_LT(getDistBetweenRobotAndGoal(goal), 0.05);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
