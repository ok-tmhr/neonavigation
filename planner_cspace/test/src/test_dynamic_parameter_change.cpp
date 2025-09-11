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

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <planner_cspace/action_test_base.h>

class DynamicParameterChangeTest
  : public ActionTestBase<nav2_msgs::action::NavigateToPose, ACTION_TOPIC_MOVE_BASE>
{
public:
  void SetUp() final
  {
    path_ = nullptr;
    planner_3d_client_.reset(
        new dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>("/planner_3d/"));
    sub_path_ = node_.subscribe("path", 1, &DynamicParameterChangeTest::cbPath, this);
    pub_map_overlay_ = node_.advertise<nav_msgs::msg::OccupancyGrid>("map_overlay", rclcpp::QoS(1).transient_local());
    pub_odom_ = node_.advertise<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1).transient_local());  // not actually used

    const rclcpp::Time deadline = this->now() + rclcpp::Duration::from_seconds(2);
    while (sub_path_.getNumPublishers() < 1 || pub_map_overlay_->get_subscription_count() < 1)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      ASSERT_TRUE(rclcpp::ok());
      ASSERT_LT(this->now(), deadline);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // wait some more time to ensure tf topic connection

    map_overlay_.header.frame_id = "map";
    map_overlay_.info.resolution = 0.1;
    map_overlay_.info.width = 32;
    map_overlay_.info.height = 32;
    map_overlay_.info.origin.position.x = 0.0;
    map_overlay_.info.origin.position.y = 0.0;
    map_overlay_.info.origin.position.z = 0.0;
    map_overlay_.info.origin.orientation.x = 0.0;
    map_overlay_.info.origin.orientation.y = 0.0;
    map_overlay_.info.origin.orientation.z = 0.0;
    map_overlay_.info.origin.orientation.w = 1.0;
    map_overlay_.data.resize(map_overlay_.info.width * map_overlay_.info.height, 0);
    publishMapAndRobot(0, 0, 0);
    ActionTestBase<nav2_msgs::action::NavigateToPose, ACTION_TOPIC_MOVE_BASE>::SetUp();

    default_config_ = planner_cspace::Planner3DConfig::__getDefault__();
    default_config_.max_retry_num = 5;
    default_config_.tolerance_range = 0.0;
    default_config_.temporary_escape = false;
    default_config_.goal_tolerance_lin = 0.05;
    default_config_.cost_in_place_turn = 3.0;
    default_config_.min_curve_radius = 0.4;
    default_config_.max_vel = 0.3;
    default_config_.max_ang_vel = 1.0;
    ASSERT_TRUE(planner_3d_client_->setConfiguration(default_config_));
  }
  void TearDown() final
  {
    move_base_->async_cancel_all_goals();
  }

protected:
  void cbPath(const nav_msgs::msg::Path::ConstPtr& msg)
  {
    path_ = msg;
    ++path_received_count_;
    last_path_received_time_ = this->now();
  }

  nav2_msgs::action::NavigateToPose::Goal CreateGoalInFree()
  {
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.target_pose.header.stamp = this->now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 1.25;
    goal.target_pose.pose.position.y = 1.05;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = std::sin(M_PI / 4);
    goal.target_pose.pose.orientation.w = std::cos(M_PI / 4);
    return goal;
  }

  bool isPathIncludingCurves() const
  {
    for (size_t i = 0; i < path_->poses.size() - 1; ++i)
    {
      const auto& pose1 = path_->poses[i];
      const auto& pose2 = path_->poses[i + 1];
      const double distance = std::hypot(pose1.pose.position.x - pose2.pose.position.x,
                                         pose1.pose.position.y - pose2.pose.position.y);
      const double yaw_diff = std::abs(tf2::getYaw(pose1.pose.orientation) - tf2::getYaw(pose2.pose.orientation));
      if (distance > 1.0e-3 && yaw_diff > 1.0e-3)
      {
        return true;
      }
    }
    return false;
  }

  ::testing::AssertionResult comparePath(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2)
  {
    if (path1.poses.size() != path2.poses.size())
    {
      return ::testing::AssertionFailure()
             << "Path size different: " << path1.poses.size() << " != " << path2.poses.size();
    }
    for (size_t i = 0; i < path1.poses.size(); ++i)
    {
      const geometry_msgs::msg::Point& pos1 = path1.poses[i].pose.position;
      const geometry_msgs::msg::Point& pos2 = path2.poses[i].pose.position;
      if (std::abs(pos1.x - pos2.x) > 1.0e-6)
      {
        return ::testing::AssertionFailure() << "X different at #" << i << ": " << pos1.x << " != " << pos2.x;
      }
      if (std::abs(pos1.y - pos2.y) > 1.0e-6)
      {
        return ::testing::AssertionFailure() << "Y different at #" << i << ": " << pos1.y << " != " << pos2.y;
      }
      const double yaw1 = tf2::getYaw(path1.poses[i].pose.orientation);
      const double yaw2 = tf2::getYaw(path2.poses[i].pose.orientation);
      if (std::abs(yaw1 - yaw2) > 1.0e-6)
      {
        return ::testing::AssertionFailure() << "Yaw different at #" << i << ": " << yaw1 << " != " << yaw2;
      }
    }
    return ::testing::AssertionSuccess();
  }

  void sendGoalAndWaitForPath()
  {
    move_base_->async_send_goal(CreateGoalInFree());

    rclcpp::spin_some(shared_from_this());  // Flush message buffer
    path_ = nullptr;

    const rclcpp::Time start_time = this->now();
    rclcpp::Time deadline = start_time + rclcpp::Duration::from_seconds(1.0);
    while (rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(shared_from_this());
      if (path_ && (path_->header.stamp > start_time) && (path_->poses.size() > 0))
      {
        break;
      }
      ASSERT_LT(this->now(), deadline)
          << "Failed to plan:" << move_base_->getState().toString() << statusString();
    }
  }

  void publishMapAndRobot(const double x, const double y, const double yaw)
  {
    const rclcpp::Time current_time = this->now();
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = current_time;
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.transform.translation = tf2::toMsg(tf2::Vector3(x, y, 0.0));
    trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    tfb_->sendTransform(trans);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = current_time;
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    pub_odom_->publish(odom);

    pub_map_overlay_->publish(map_overlay_);
  }

  double getAveragePathInterval(const rclcpp::Duration& costmap_publishing_interval)
  {
    publishMapAndRobot(2.55, 0.45, M_PI);
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    move_base_->async_send_goal(CreateGoalInFree());
    while (rclcpp::ok() && (move_base_->getState() != rclcpp_action::ResultCode::ACTIVE))
    {
      rclcpp::spin_some(shared_from_this());
    }

    last_path_received_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publishMapAndRobot(2.55, 0.45, M_PI);
    rclcpp::Time last_costmap_publishing_time = this->now();
    rclcpp::Rate r(100);
    while (rclcpp::ok() && (last_path_received_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)))
    {
      if ((this->now() - last_costmap_publishing_time) > costmap_publishing_interval)
      {
        publishMapAndRobot(2.55, 0.45, M_PI);
        last_costmap_publishing_time = this->now();
      };
      rclcpp::spin_some(shared_from_this());
      r.sleep();
    }
    const rclcpp::Time initial_path_received_time_ = last_path_received_time_;
    const int prev_path_received_count = path_received_count_;
    while (rclcpp::ok() && (path_received_count_ < prev_path_received_count + 10))
    {
      if ((this->now() - last_costmap_publishing_time) > costmap_publishing_interval)
      {
        publishMapAndRobot(2.55, 0.45, M_PI);
        last_costmap_publishing_time = this->now();
      }
      rclcpp::spin_some(shared_from_this());
      r.sleep();
    }
    return (last_path_received_time_ - initial_path_received_time_).seconds() /
           (path_received_count_ - prev_path_received_count);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Subscription<>::SharedPtr sub_path_;
  nav_msgs::msg::Path::ConstPtr path_;
  std::unique_ptr<dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>> planner_3d_client_;
  rclcpp::Publisher<>::SharedPtr pub_map_overlay_;
  rclcpp::Publisher<>::SharedPtr pub_odom_;
  nav_msgs::msg::OccupancyGrid map_overlay_;
  planner_cspace::Planner3DConfig default_config_;
  int path_received_count_;
  rclcpp::Time last_path_received_time_;
};

TEST_F(DynamicParameterChangeTest, DisableCurves)
{
  publishMapAndRobot(2.55, 0.45, M_PI);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  sendGoalAndWaitForPath();
  // The default path is including curves.
  EXPECT_TRUE(isPathIncludingCurves());

  planner_cspace::Planner3DConfig config = default_config_;
  // Large min_curve_radius disables curves.
  config.min_curve_radius = 10.0;
  ASSERT_TRUE(planner_3d_client_->setConfiguration(config));

  sendGoalAndWaitForPath();
  // The default path is including curves.
  EXPECT_FALSE(isPathIncludingCurves());
}

TEST_F(DynamicParameterChangeTest, StartPosePrediction)
{
  publishMapAndRobot(1.65, 0.65, M_PI);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  sendGoalAndWaitForPath();
  const nav_msgs::msg::Path initial_path = *path_;

  // The path is changed to keep distance from the obstacle.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 100;
  publishMapAndRobot(1.65, 0.65, M_PI);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  sendGoalAndWaitForPath();
  EXPECT_FALSE(comparePath(initial_path, *path_));

  // Enable start pose prediction.
  move_base_->async_cancel_all_goals();
  planner_cspace::Planner3DConfig config = default_config_;
  config.keep_a_part_of_previous_path = true;
  config.dist_stop_to_previous_path = 0.1;
  ASSERT_TRUE(planner_3d_client_->setConfiguration(config));

  // No obstacle and the path is same as the first one.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 0;
  publishMapAndRobot(1.65, 0.65, M_PI);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(initial_path, *path_));

  // The path does not change as a part of the previous path is kept and it is not possible to keep distance from
  // the obstacle.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 100;
  publishMapAndRobot(1.65, 0.65, M_PI);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(initial_path, *path_));

  // It is expected that the robot reaches the goal during the path planning.
  move_base_->async_cancel_all_goals();
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 0;
  publishMapAndRobot(1.25, 0.95, M_PI / 2);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  sendGoalAndWaitForPath();
  const nav_msgs::msg::Path short_path = *path_;
  // In the second path planning after cancel, the exptected start pose is same as the goal.
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(short_path, *path_));
}

TEST_F(DynamicParameterChangeTest, TriggerPlanByCostmapUpdate)
{
  publishMapAndRobot(2.55, 0.45, M_PI);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  sendGoalAndWaitForPath();

  const rclcpp::Duration costmap_publishing_interval(0.1);
  // The path planning frequency is 4.0 Hz (Designated by the "freq" paramteer)
  const double default_interval = getAveragePathInterval(costmap_publishing_interval);
  EXPECT_NEAR(default_interval, 1.0 / default_config_.freq, (1.0 / default_config_.freq) * 0.1);

  planner_cspace::Planner3DConfig config = default_config_;
  config.trigger_plan_by_costmap_update = true;
  config.costmap_watchdog = 0.5;
  ASSERT_TRUE(planner_3d_client_->setConfiguration(config));

  // The path planning is trigger by the callback of CSpace3DUpdate, so its frequency is same as the frequency of
  // CSpace3DUpdate (10 Hz).
  const double interval_triggered_by_costmap = getAveragePathInterval(costmap_publishing_interval);
  EXPECT_NEAR(interval_triggered_by_costmap, costmap_publishing_interval.seconds(),
              costmap_publishing_interval.seconds() * 0.1);

  // The path planning is trigger by costmap_watchdog_(0.5 seconds) when CSpace3DUpdate is not published.
  const double interval_triggered_by_watchdog = getAveragePathInterval(rclcpp::Duration::from_seconds(100));
  EXPECT_NEAR(interval_triggered_by_watchdog, config.costmap_watchdog, config.costmap_watchdog * 0.1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv, "test_dynamic_parameter_change");
  return RUN_ALL_TESTS();
}
