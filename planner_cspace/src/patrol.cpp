/*
 * Copyright (c) 2016-2017, the neonavigation authors
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

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <planner_cspace_msgs/action/move_with_tolerance.hpp>
#include <nav_msgs/msg/path.hpp>


class PatrolActionNode : public rclcpp::Node
{
protected:
  using MoveBaseClient = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;
  using MoveWithToleranceClient = rclcpp_action::Client<planner_cspace_msgs::action::MoveWithTolerance>;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  std::shared_ptr<MoveBaseClient> act_cli_;
  std::shared_ptr<MoveWithToleranceClient> act_cli_tolerant_;

  nav_msgs::msg::Path path_;
  size_t pos_;
  bool with_tolerance_;
  double tolerance_lin_;
  double tolerance_ang_;
  double tolerance_ang_finish_;

  void cbPath(const nav_msgs::msg::Path::ConstPtr& msg)
  {
    if (path_.poses.size() > 0)
    {
      // Cancel previous patrol if stored
      if (with_tolerance_)
      {
        act_cli_tolerant_->async_cancel_all_goals();
      }
      else
      {
        act_cli_->async_cancel_all_goals();
      }
    }
    path_ = *msg;
    pos_ = 0;
  }

public:
  using GoalHandleMoveBase = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using GoalHandleMoveWithTolerance = rclcpp_action::ClientGoalHandle<planner_cspace_msgs::action::MoveWithTolerance>;
  PatrolActionNode()
    : rclcpp::Node("patrol")
  {

    using std::placeholders::_1;
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "patrol_nodes",
        1, std::bind(&PatrolActionNode::cbPath, this, _1));

    with_tolerance_ = this->declare_parameter("with_tolerance", false);
    tolerance_lin_ = this->declare_parameter("tolerance_lin", 0.1);
    tolerance_ang_ = this->declare_parameter("tolerance_ang", 0.1);
    tolerance_ang_finish_ = this->declare_parameter("tolerance_ang_finish", 0.05);

    if (with_tolerance_)
    {
      act_cli_tolerant_ = rclcpp_action::create_client<planner_cspace_msgs::action::MoveWithTolerance>(
        this, "tolerant_move");
    }
    else
    {
      act_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "move_base");
    }

    pos_ = 0;
  }
  void on_move_with_tolerance_result(const GoalHandleMoveWithTolerance::WrappedResult& result){
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Action has been finished.");
      sendNextGoal();
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_ERROR(this->get_logger(), "Action has been aborted. Skipping.");
      sendNextGoal();
    }
    else if (result.code == rclcpp_action::ResultCode::UNKNOWN)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Action server is not ready.");
    }
  }
  void on_move_base_result(const GoalHandleMoveBase::WrappedResult& result){
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Action has been finished.");
      sendNextGoal();
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_ERROR(this->get_logger(), "Action has been aborted. Skipping.");
      sendNextGoal();
    }
    else if (result.code == rclcpp_action::ResultCode::UNKNOWN)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Action server is not ready.");
    }
  }
  bool sendNextGoal()
  {
    if (path_.poses.size() <= pos_)
    {
      RCLCPP_WARN(this->get_logger(), "Patrol finished. Waiting next path.");
      path_.poses.clear();

      return false;
    }

    using std::placeholders::_1;
    if (with_tolerance_)
    {
      planner_cspace_msgs::action::MoveWithTolerance_Goal goal;

      goal.target_pose.header = path_.poses[pos_].header;
      goal.target_pose.header.stamp = now();
      goal.target_pose.pose = path_.poses[pos_].pose;
      goal.goal_tolerance_lin = tolerance_lin_;
      goal.goal_tolerance_ang = tolerance_ang_;
      goal.goal_tolerance_ang_finish = tolerance_ang_finish_;

      auto goal_options = rclcpp_action::Client<planner_cspace_msgs::action::MoveWithTolerance>::SendGoalOptions();
      goal_options.result_callback = std::bind(&PatrolActionNode::on_move_with_tolerance_result, this, _1);
      act_cli_tolerant_->async_send_goal(goal, goal_options);
    }
    else
    {
      nav2_msgs::action::NavigateToPose_Goal goal;

      goal.pose.header = path_.poses[pos_].header;
      goal.pose.header.stamp = now();
      goal.pose.pose = path_.poses[pos_].pose;

      auto goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      goal_options.result_callback = std::bind(&PatrolActionNode::on_move_base_result, this, _1);
      act_cli_->async_send_goal(goal);
    }
    pos_++;

    return true;
  }
  void spin()
  {
    rclcpp::Rate rate(10.0);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());
      rate.sleep();

      if (path_.poses.size() == 0)
      {
        continue;
      }

      if (pos_ == 0)
      {
        sendNextGoal();
        continue;
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto pa = std::make_shared<PatrolActionNode>();
  pa->spin();

  return 0;
}
