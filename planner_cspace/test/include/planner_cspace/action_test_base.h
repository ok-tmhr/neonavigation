/*
 * Copyright (c) 2018-2023, the neonavigation authors
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

#ifndef PLANNER_CSPACE_ACTION_TEST_BASE_H
#define PLANNER_CSPACE_ACTION_TEST_BASE_H

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <actionlib/client/simple_action_client.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <planner_cspace_msgs/msg/planner_status.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

constexpr const char ACTION_TOPIC_MOVE_BASE[] = "/move_base";
constexpr const char ACTION_TOPIC_TOLERANT_MOVE[] = "/tolerant_move";

template <typename ACTION, char const* TOPIC>
class ActionTestBase : public ::testing::Test
{
public:
  ActionTestBase()
    : tfl_(tfbuf_)
    , map_ready_(false)
  {
    move_base_ = std::make_shared<ActionClient>(TOPIC);
    sub_status_ = node_.subscribe(
        "/planner_3d/status", 10, &ActionTestBase::cbStatus, this);
  }
  void SetUp()
  {
    if (!move_base_->waitForServer(rclcpp::Duration(30.0)))
    {
      FAIL() << "Failed to connect move_base action";
    }

    rclcpp::ServiceClient srv_plan =
        node_.serviceClient<nav_msgs::srv::GetPlanRequest, nav_msgs::srv::GetPlanResponse>(
            "/planner_3d/make_plan");

    const rclcpp::Time deadline = this->now() + rclcpp::Duration(10.0);
    while (rclcpp::ok())
    {
      nav_msgs::srv::GetPlanRequest req;
      nav_msgs::srv::GetPlanResponse res;
      req.tolerance = 10.0;
      req.start.header.frame_id = "map";
      req.start.pose.position.x = 1.24;
      req.start.pose.position.y = 0.65;
      req.start.pose.orientation.w = 1;
      req.goal.header.frame_id = "map";
      req.goal.pose.position.x = 1.25;
      req.goal.pose.position.y = 0.75;
      req.goal.pose.orientation.w = 1;
      if (srv_plan.call(req, res))
      {
        // Planner is ready.
        break;
      }
      if (this->now() > deadline)
      {
        FAIL() << "planner_3d didn't receive map";
      }
      rclcpp::Duration(1).sleep();
      rclcpp::spin_some(shared_from_this());
    }
  }
  ~ActionTestBase()
  {
  }

protected:
  using ActionClient = actionlib::SimpleActionClient<ACTION>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;

  void cbStatus(const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg)
  {
    planner_status_ = msg;
  }

  std::string statusString() const
  {
    if (!planner_status_)
    {
      return "(no status)";
    }
    return "(status: " + std::to_string(planner_status_->status) +
           ", error: " + std::to_string(planner_status_->error) + ")";
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<>::SharedPtr sub_status_;
  ActionClientPtr move_base_;
  planner_cspace_msgs::msg::PlannerStatus::ConstPtr planner_status_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  bool map_ready_;
};

#endif  // PLANNER_CSPACE_ACTION_TEST_BASE_H
