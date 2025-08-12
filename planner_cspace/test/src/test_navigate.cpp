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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <costmap_cspace_msgs/msg/c_space3_d.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <planner_cspace_msgs/msg/planner_status.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>

#include <planner_cspace/planner_status.h>

#include <gtest/gtest.h>

class Navigate : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr nh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  nav_msgs::msg::OccupancyGrid::ConstPtr map_;
  nav_msgs::msg::OccupancyGrid::Ptr map_local_;
  planner_cspace_msgs::msg::PlannerStatus::ConstPtr planner_status_;
  costmap_cspace_msgs::msg::CSpace3D::ConstPtr costmap_;
  nav_msgs::msg::Path::ConstPtr path_;
  trajectory_tracker_msgs::msg::PathWithVelocity::ConstPtr path_vel_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_local_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr sub_costmap_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<trajectory_tracker_msgs::msg::PathWithVelocity>::SharedPtr sub_path_vel_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_forget_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_local_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_patrol_nodes_;
  size_t local_map_apply_cnt_;
  std::vector<tf2::Stamped<tf2::Transform>> traj_;
  std::string test_scope_;

  Navigate()
    : nh_(rclcpp::Node::make_shared("test_navigate"))
    , tfbuf_(nh_->get_clock())
    , tfl_(tfbuf_)
    , local_map_apply_cnt_(0)
  {
    using std::placeholders::_1;
    sub_map_ = nh_->create_subscription<nav_msgs::msg::OccupancyGrid>("map_global", 1, std::bind(&Navigate::cbMap, this, _1));
    sub_map_local_ = nh_->create_subscription<nav_msgs::msg::OccupancyGrid>("map_local", 1, std::bind(&Navigate::cbMapLocal, this, _1));
    sub_costmap_ = nh_->create_subscription<costmap_cspace_msgs::msg::CSpace3D>("costmap", 1, std::bind(&Navigate::cbCostmap, this, _1));
    sub_status_ = nh_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
        "/planner_3d/status", 10, std::bind(&Navigate::cbStatus, this, _1));
    sub_path_ = nh_->create_subscription<nav_msgs::msg::Path>("path", 1, std::bind(&Navigate::cbPath, this, _1));
    sub_path_vel_ = nh_->create_subscription<trajectory_tracker_msgs::msg::PathWithVelocity>("path_velocity", 1, std::bind(&Navigate::cbPathVel, this, _1));
    srv_forget_ =
        nh_->create_client<std_srvs::srv::Empty>(
            "forget_planning_cost");
    pub_map_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
    pub_map_local_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("overlay", rclcpp::QoS(1).transient_local());
    pub_initial_pose_ =
        nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(1).transient_local());
    pub_patrol_nodes_ = nh_->create_publisher<nav_msgs::msg::Path>("patrol_nodes", rclcpp::QoS(1).transient_local());
  }

  void SetUp() override
  {
    test_scope_ =
        "[" + std::to_string(getpid()) + "/" +
        ::testing::UnitTest::GetInstance()->current_test_info()->name() + "] ";

    rclcpp::Rate rate(10.0);

    const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(15.0);
    while (rclcpp::ok())
    {
      rate.sleep();
      ASSERT_LT(nh_->now(), deadline)
          << test_scope_ << "Initialization timeout: "
          << "sub_map:" << sub_map_->get_publisher_count() << " "
          << "sub_map_local:" << sub_map_local_->get_publisher_count() << " "
          << "sub_costmap:" << sub_costmap_->get_publisher_count() << " "
          << "sub_status:" << sub_status_->get_publisher_count() << " "
          << "pub_map:" << pub_map_->get_subscription_count() << " "
          << "pub_map_local:" << pub_map_local_->get_subscription_count() << " "
          << "pub_initial_pose:" << pub_initial_pose_->get_subscription_count() << " "
          << "pub_patrol_nodes:" << pub_patrol_nodes_->get_subscription_count() << " ";
      if (sub_map_->get_publisher_count() > 0 &&
          sub_map_local_->get_publisher_count() > 0 &&
          sub_costmap_->get_publisher_count() > 0 &&
          sub_status_->get_publisher_count() > 0 &&
          pub_map_->get_subscription_count() > 0 &&
          pub_map_local_->get_subscription_count() > 0 &&
          pub_initial_pose_->get_subscription_count() > 0 &&
          pub_patrol_nodes_->get_subscription_count() > 0)
      {
        break;
      }
    }
    ASSERT_TRUE(srv_forget_->wait_for_service(std::chrono::duration<double>(10.0)));

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 2.5;
    pose.pose.pose.position.y = 0.45;
    pose.pose.pose.orientation.z = 1.0;
    pub_initial_pose_->publish(pose);

    while (rclcpp::ok())
    {
      rclcpp::spin_some(nh_);
      rate.sleep();
      const rclcpp::Time now = nh_->now();
      ASSERT_LT(now, deadline) << test_scope_ << "Initial transform timeout";
      if (tfbuf_.canTransform("map", "base_link", now, rclcpp::Duration::from_seconds(0.5)))
      {
        break;
      }
    }

    while (rclcpp::ok() && !map_)
    {
      rclcpp::spin_some(nh_);
      rate.sleep();
      ASSERT_LT(nh_->now(), deadline) << test_scope_ << "Initial map timeout";
    }
    pub_map_->publish(*map_);
    std::cerr << test_scope_ << nh_->now().seconds() << " Map applied." << std::endl;

    while (rclcpp::ok() && !map_local_)
    {
      rclcpp::spin_some(nh_);
      rate.sleep();
      ASSERT_LT(nh_->now(), deadline) << test_scope_ << "Initial local map timeout";
    }

    while (rclcpp::ok() && !costmap_)
    {
      rclcpp::spin_some(nh_);
      rate.sleep();
      ASSERT_LT(nh_->now(), deadline) << test_scope_ << "Initial costmap timeout";
    }

    std_srvs::srv::Empty_Request::SharedPtr req;
    std_srvs::srv::Empty_Response::SharedPtr res;
    auto future = srv_forget_->async_send_request(req);
    res = future.get();

    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  void TearDown() override
  {
    // Clear goal
    nav_msgs::msg::Path path;
    pub_patrol_nodes_->publish(path);
    rclcpp::sleep_for(std::chrono::seconds(2));
  }
  void cbCostmap(const costmap_cspace_msgs::msg::CSpace3D::ConstPtr& msg)
  {
    costmap_ = msg;
    std::cerr << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " Costmap received." << std::endl;
  }
  void cbMap(const nav_msgs::msg::OccupancyGrid::ConstPtr& msg)
  {
    map_ = msg;
    std::cerr << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " Map received." << std::endl;
  }
  void cbMapLocal(const nav_msgs::msg::OccupancyGrid::ConstPtr& msg)
  {
    if (map_local_)
    {
      return;
    }
    map_local_.reset(new nav_msgs::msg::OccupancyGrid(*msg));
    std::cerr << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " Local map received." << std::endl;
  }
  void cbStatus(const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg)
  {
    if (!planner_status_ || planner_status_->status != msg->status || planner_status_->error != msg->error)
    {
      std::cerr << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " Status updated." << msg << std::endl;
    }
    planner_status_ = msg;
  }
  void cbPath(const nav_msgs::msg::Path::ConstPtr& msg)
  {
    if (!path_ || path_->poses.size() != msg->poses.size())
    {
      if (msg->poses.size() == 0)
      {
        std::cerr << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " Path updated. (empty)" << std::endl;
      }
      else
      {
        std::cerr
            << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " Path updated." << std::endl
            << msg->poses.front().pose.position.x << ", " << msg->poses.front().pose.position.y << std::endl
            << msg->poses.back().pose.position.x << ", " << msg->poses.back().pose.position.y << std::endl;
      }
      path_ = msg;
    }
  }
  void cbPathVel(const trajectory_tracker_msgs::msg::PathWithVelocity::ConstPtr& msg)
  {
    if (!path_vel_ || path_vel_->poses.size() != msg->poses.size())
    {
      if (msg->poses.size() == 0)
      {
        std::cerr << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " PathWithVelocity updated. (empty)" << std::endl;
      }
      else
      {
        std::cerr
            << test_scope_ << rclcpp::Time(msg->header.stamp).seconds() << " PathWithVelocity updated." << std::endl
            << msg->poses.front().pose.position.x << ", " << msg->poses.front().pose.position.y << std::endl
            << msg->poses.back().pose.position.x << ", " << msg->poses.back().pose.position.y << std::endl;
      }
      path_vel_ = msg;
    }
  }
  void pubMapLocal()
  {
    if (!map_local_)
    {
      return;
    }
    pub_map_local_->publish(*map_local_);
    if ((local_map_apply_cnt_++) % 30 == 0)
    {
      int num_occupied = 0;
      for (const auto& c : map_local_->data)
      {
        if (c == 100)
        {
          num_occupied++;
        }
      }
      std::cerr << test_scope_ << " Local map applied. occupied grids:" << num_occupied << std::endl;
    }
  }
  tf2::Stamped<tf2::Transform> lookupRobotTrans(const rclcpp::Time& now)
  {
    geometry_msgs::msg::TransformStamped trans_tmp =
        tfbuf_.lookupTransform("map", "base_link", now, rclcpp::Duration::from_seconds(0.5));
    tf2::Stamped<tf2::Transform> trans;
    tf2::fromMsg(trans_tmp, trans);
    traj_.push_back(trans);
    return trans;
  }
  void dumpRobotTrajectory()
  {
    double x_prev(0), y_prev(0);
    tf2::Quaternion rot_prev(0, 0, 0, 1);

    std::cerr << test_scope_ << traj_.size() << " points recorded" << std::endl;

    for (const auto& t : traj_)
    {
      const double x = t.getOrigin().getX();
      const double y = t.getOrigin().getY();
      const tf2::Quaternion rot = t.getRotation();
      const double yaw_diff = rot.angleShortestPath(rot_prev);
      if (std::abs(x - x_prev) >= 0 || std::abs(y - y_prev) >= 0 || std::abs(yaw_diff) >= 0)
      {
        x_prev = x;
        y_prev = y;
        rot_prev = rot;
        std::cerr << tf2::timeToSec(t.stamp_) << " " << x << " " << y << " " << tf2::getYaw(rot) << std::endl;
      }
    }
  }

  void waitForPlannerStatus(const std::string& name, const int expected_error)
  {
    rclcpp::spin_some(nh_);
    ASSERT_TRUE(static_cast<bool>(map_));
    ASSERT_TRUE(static_cast<bool>(map_local_));
    pubMapLocal();
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    rclcpp::Rate wait(10);
    rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(10);
    while (rclcpp::ok())
    {
      pubMapLocal();
      rclcpp::spin_some(nh_);
      wait.sleep();

      const rclcpp::Time now = nh_->now();

      if (now > deadline)
      {
        dumpRobotTrajectory();
        FAIL()
            << test_scope_ << "/" << name << ": Navigation timeout." << std::endl
            << "now: " << now.seconds() << std::endl
            << "status: " << planner_status_ << " (expected: " << expected_error << ")";
      }

      if (planner_status_->error == expected_error)
      {
        return;
      }
    }
  }
};

TEST_F(Navigate, Navigate)
{
  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));

  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.7;
  path.poses[0].pose.position.y = 2.8;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  path.poses[1].header.frame_id = path.header.frame_id;
  path.poses[1].pose.position.x = 1.9;
  path.poses[1].pose.position.y = 2.8;
  path.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -1.57));
  pub_patrol_nodes_->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(10);
  const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(60);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now.seconds() << std::endl
          << "status: " << planner_status_;
      break;
    }

    tf2::Stamped<tf2::Transform> trans;
    try
    {
      trans = lookupRobotTrans(now);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2)
    {
      std::cerr << test_scope_ << "Navigation success." << std::endl;
      return;
    }

    for (int x = -2; x <= 2; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        const tf2::Vector3 pos =
            trans * tf2::Vector3(x * map_->info.resolution, y * map_->info.resolution, 0);
        const int map_x = pos.x() / map_->info.resolution;
        const int map_y = pos.y() / map_->info.resolution;
        const size_t addr = map_x + map_y * map_->info.width;
        ASSERT_LT(addr, map_->data.size());
        ASSERT_LT(map_x, static_cast<int>(map_->info.width));
        ASSERT_LT(map_y, static_cast<int>(map_->info.height));
        ASSERT_GE(map_x, 0);
        ASSERT_GE(map_y, 0);
        ASSERT_NE(map_->data[addr], 100);
      }
    }
  }
  ASSERT_TRUE(false);
}

TEST_F(Navigate, NavigateWithLocalMap)
{
  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));
  pubMapLocal();
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.7;
  path.poses[0].pose.position.y = 2.8;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  pub_patrol_nodes_->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(10);
  const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(60);
  while (rclcpp::ok())
  {
    pubMapLocal();
    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now.seconds() << std::endl
          << "status: " << planner_status_;
      break;
    }

    tf2::Stamped<tf2::Transform> trans;
    try
    {
      trans = lookupRobotTrans(now);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2)
    {
      std::cerr << test_scope_ << "Navagation success." << std::endl;
      return;
    }

    for (int x = -2; x <= 2; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        const tf2::Vector3 pos =
            trans * tf2::Vector3(x * map_->info.resolution, y * map_->info.resolution, 0);
        const int map_x = pos.x() / map_->info.resolution;
        const int map_y = pos.y() / map_->info.resolution;
        const size_t addr = map_x + map_y * map_->info.width;
        ASSERT_LT(addr, map_->data.size());
        ASSERT_LT(map_x, static_cast<int>(map_->info.width));
        ASSERT_LT(map_y, static_cast<int>(map_->info.height));
        ASSERT_GE(map_x, 0);
        ASSERT_GE(map_y, 0);
        ASSERT_NE(map_->data[addr], 100);
        ASSERT_NE(map_local_->data[addr], 100);
      }
    }
  }
  ASSERT_TRUE(false);
}

TEST_F(Navigate, GlobalPlan)
{
  auto srv_plan =
      nh_->create_client<nav_msgs::srv::GetPlan>(
          "/planner_3d/make_plan");

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));

  nav_msgs::srv::GetPlan_Request::SharedPtr req;
  nav_msgs::srv::GetPlan_Response::SharedPtr res;

  req->tolerance = 0.0;
  req->start.header.frame_id = "map";
  req->start.pose.position.x = 1.95;
  req->start.pose.position.y = 0.45;
  req->start.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 3.14));

  req->goal.header.frame_id = "map";
  req->goal.pose.position.x = 1.25;
  req->goal.pose.position.y = 2.15;
  req->goal.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  // Planning failes as (12, 21, 0) is in rock.
  ASSERT_FALSE(rclcpp::spin_until_future_complete(nh_, srv_plan->async_send_request(req)) == rclcpp::FutureReturnCode::SUCCESS);

  // Goal grid is moved to (12, 22, 0).
  req->tolerance = 0.1;
  ASSERT_TRUE(rclcpp::spin_until_future_complete(nh_, srv_plan->async_send_request(req)) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_NEAR(1.25, res->plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(2.25, res->plan.poses.back().pose.position.y, 1.0e-5);

  // Goal grid is moved to (12, 23, 0). This is because cost of (12, 22, 0) is larger than 50.
  req->tolerance = 0.2f;
  ASSERT_TRUE(rclcpp::spin_until_future_complete(nh_, srv_plan->async_send_request(req)) == rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_NEAR(1.25, res->plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(2.35, res->plan.poses.back().pose.position.y, 1.0e-5);

  req->tolerance = 0.0;
  req->goal.header.frame_id = "map";
  req->goal.pose.position.x = 1.85;
  req->goal.pose.position.y = 2.75;
  req->goal.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -1.57));
  ASSERT_TRUE(rclcpp::spin_until_future_complete(nh_, srv_plan->async_send_request(req)) == rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_NEAR(req->start.pose.position.x, res->plan.poses.front().pose.position.x, 1.0e-5);
  EXPECT_NEAR(req->start.pose.position.y, res->plan.poses.front().pose.position.y, 1.0e-5);
  EXPECT_NEAR(req->goal.pose.position.x, res->plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(req->goal.pose.position.y, res->plan.poses.back().pose.position.y, 1.0e-5);

  for (const geometry_msgs::msg::PoseStamped& p : res->plan.poses)
  {
    const int map_x = p.pose.position.x / map_->info.resolution;
    const int map_y = p.pose.position.y / map_->info.resolution;
    const size_t addr = map_x + map_y * map_->info.width;
    ASSERT_LT(addr, map_->data.size());
    ASSERT_LT(map_x, static_cast<int>(map_->info.width));
    ASSERT_LT(map_y, static_cast<int>(map_->info.height));
    ASSERT_GE(map_x, 0);
    ASSERT_GE(map_y, 0);
    ASSERT_NE(map_->data[addr], 100);
  }
}

TEST_F(Navigate, RobotIsInRockOnSetGoal)
{
  auto pub_path = nh_->create_publisher<nav_msgs::msg::Path>("patrol_nodes", rclcpp::QoS(1).transient_local());

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));
  pubMapLocal();
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.19;
  path.poses[0].pose.position.y = 1.90;
  path.poses[0].pose.orientation.w = 1.0;
  pub_path->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(10);
  const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(10);
  while (rclcpp::ok())
  {
    pubMapLocal();
    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now.seconds() << std::endl
          << "status: " << planner_status_;
      break;
    }

    if (planner_status_->error == planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND)
    {
      return;
    }
  }
  ASSERT_TRUE(false);
}

TEST_F(Navigate, GoalIsInRockRecovered)
{
  if (nh_->declare_parameter("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is set";
  }

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  for (int x = 10; x <= 16; ++x)
  {
    for (int y = 22; y <= 26; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 100;
    }
  }
  pubMapLocal();

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.25;
  path.poses[0].pose.position.y = 2.55;
  path.poses[0].pose.orientation.w = 1.0;
  pub_patrol_nodes_->publish(path);

  waitForPlannerStatus("Got stuck", planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
  ASSERT_FALSE(::testing::Test::HasFailure());

  for (int x = 10; x <= 16; ++x)
  {
    for (int y = 22; y <= 26; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 0;
    }
  }
  waitForPlannerStatus("Stuck recovered", planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);
}

TEST_F(Navigate, RobotIsInRockOnRecovered)
{
  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  for (int x = 21; x <= 28; ++x)
  {
    for (int y = 2; y <= 8; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 100;
    }
  }
  pubMapLocal();

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.25;
  path.poses[0].pose.position.y = 2.55;
  path.poses[0].pose.orientation.w = 1.0;
  pub_patrol_nodes_->publish(path);

  waitForPlannerStatus("Got stuck", planner_cspace_msgs::msg::PlannerStatus::IN_ROCK);
  ASSERT_FALSE(::testing::Test::HasFailure());

  for (int x = 21; x <= 28; ++x)
  {
    for (int y = 2; y <= 8; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 0;
    }
  }
  waitForPlannerStatus("Stuck recovered", planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);
}

TEST_F(Navigate, CrowdEscapeOnSurrounded)
{
  if (!nh_->declare_parameter("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.6;
  path.poses[0].pose.position.y = 2.2;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
  pub_patrol_nodes_->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(10);
  const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(60);
  while (rclcpp::ok())
  {
    pubMapLocal();
    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now.seconds() << std::endl
          << "status: " << planner_status_;
      break;
    }

    tf2::Stamped<tf2::Transform> trans;
    try
    {
      trans = lookupRobotTrans(now);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);

    const auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2 &&
        planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DONE)
    {
      std::cerr << test_scope_ << "Navigation success." << std::endl;
      return;
    }

    const size_t rx = trans.getOrigin().x() / map_->info.resolution;
    const size_t ry = trans.getOrigin().y() / map_->info.resolution;
    const size_t data_size = map_local_->data.size();
    map_local_->data.clear();
    map_local_->data.resize(data_size, 0);
    const int bs = 6;
    for (int i = -bs; i <= bs; ++i)
    {
      map_local_->data[std::min((rx + i) + (ry - bs) * map_local_->info.width, data_size - 1)] = 100;
      map_local_->data[std::min((rx + i) + (ry + bs) * map_local_->info.width, data_size - 1)] = 100;
      map_local_->data[std::min((rx - bs) + (ry + i) * map_local_->info.width, data_size - 1)] = 100;
      map_local_->data[std::min((rx + bs) + (ry + i) * map_local_->info.width, data_size - 1)] = 100;
    }
  }
}

TEST_F(Navigate, CrowdEscapeOnPathNotFound)
{
  if (!nh_->declare_parameter("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.6;
  path.poses[0].pose.position.y = 2.2;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
  pub_patrol_nodes_->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(10);
  bool unreachable = false;
  const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(60);
  rclcpp::Time check_until = deadline;
  while (rclcpp::ok())
  {
    const size_t data_size = map_local_->data.size();
    for (int x = 0; x < map_local_->info.width; ++x)
    {
      const size_t y = 1.1 / map_->info.resolution;
      map_local_->data[x + y * map_local_->info.width] = 100;
    }
    pubMapLocal();

    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now.seconds() << std::endl
          << "status: " << planner_status_;
      break;
    }

    if (planner_status_->error == planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND && !unreachable)
    {
      unreachable = true;
      check_until = now + rclcpp::Duration::from_seconds(2);  // Check another 2 seconds that state is not changed
    }
    if (unreachable)
    {
      EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
    }
    if (now > check_until)
    {
      return;
    }
  }
}

TEST_F(Navigate, CrowdEscapeOnGoalIsInRock)
{
  if (!nh_->declare_parameter("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.5;
  path.poses[0].pose.position.y = 0.45;
  path.poses[0].pose.orientation.z = 1.0;
  pub_patrol_nodes_->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(10);
  bool unreachable = false;
  const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(60);
  rclcpp::Time check_until = deadline;
  while (rclcpp::ok())
  {
    const size_t data_size = map_local_->data.size();
    const size_t gx = path.poses[0].pose.position.x / map_->info.resolution;
    const size_t gy = path.poses[0].pose.position.y / map_->info.resolution;
    for (int x = gx - 2; x <= gx + 2; ++x)
    {
      for (int y = gy - 2; y <= gy + 2; ++y)
      {
        map_local_->data[x + y * map_local_->info.width] = 100;
      }
    }
    pubMapLocal();

    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now.seconds() << std::endl
          << "status: " << planner_status_;
      break;
    }

    if (planner_status_->error == planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND && !unreachable)
    {
      unreachable = true;
      check_until = now + rclcpp::Duration::from_seconds(2);  // Check another 2 seconds that state is not changed
    }
    if (unreachable)
    {
      EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
    }
    if (now > check_until)
    {
      return;
    }
  }
}

TEST_F(Navigate, CrowdEscapeButNoValidTemporaryGoal)
{
  if (!nh_->declare_parameter("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.5;
  path.poses[0].pose.position.y = 0.45;
  path.poses[0].pose.orientation.z = 1.0;
  pub_patrol_nodes_->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(10);
  bool unreachable = false;
  const rclcpp::Time check_until = nh_->now() + rclcpp::Duration::from_seconds(2);
  int cnt_planning = 0;
  while (rclcpp::ok())
  {
    const size_t gx = path.poses[0].pose.position.x / map_->info.resolution;
    const size_t gy = path.poses[0].pose.position.y / map_->info.resolution;
    map_local_->data.clear();
    map_local_->data.resize(map_local_->info.width * map_local_->info.height, 60);
    for (int x = gx - 2; x <= gx + 2; ++x)
    {
      for (int y = gy - 2; y <= gy + 2; ++y)
      {
        map_local_->data[x + y * map_local_->info.width] = 100;
      }
    }
    pubMapLocal();

    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DOING)
    {
      if (cnt_planning > 1)
      {
        // Temporary goal is selected from cells with cost<50 by default.
        // No temporary goal should be selected on this map.
        EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
      }
      cnt_planning++;
    }
    if (now > check_until)
    {
      EXPECT_GT(cnt_planning, 2);
      return;
    }
  }
}

TEST_F(Navigate, ForceTemporaryEscape)
{
  if (!nh_->declare_parameter("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  auto pub_trigger = nh_->create_publisher<std_msgs::msg::Empty>("/planner_3d/temporary_escape", 1);

  rclcpp::spin_some(nh_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  nav_msgs::msg::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.6;
  path.poses[0].pose.position.y = 2.2;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
  pub_patrol_nodes_->publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::Rate wait(2);
  const rclcpp::Time deadline = nh_->now() + rclcpp::Duration::from_seconds(60);
  while (rclcpp::ok())
  {
    const size_t data_size = map_local_->data.size();
    map_local_->data.clear();
    map_local_->data.resize(data_size, 0);
    pubMapLocal();

    std_msgs::msg::Empty msg;
    pub_trigger->publish(msg);

    rclcpp::spin_some(nh_);
    wait.sleep();

    const rclcpp::Time now = nh_->now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now.seconds() << std::endl
          << "status: " << planner_status_;
      break;
    }

    tf2::Stamped<tf2::Transform> trans;
    try
    {
      trans = lookupRobotTrans(now);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);

    const auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2 &&
        planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DONE)
    {
      std::cerr << test_scope_ << "Navigation success." << std::endl;
      return;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
