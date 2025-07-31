/*
 * Copyright (c) 2022, the neonavigation authors
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
#include <cstddef>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <costmap_cspace_msgs/msg/c_space3_d.hpp>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <planner_cspace_msgs/msg/planner_status.hpp>

#include <gtest/gtest.h>

class Planner3DMapSize : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_status_;
  ros::Publisher pub_map_;
  ros::Publisher pub_map_update_;
  size_t cnt_status_;

  Planner3DMapSize()
    : cnt_status_(0)
  {
    sub_status_ = nh_.subscribe("/planner_3d/status", 100, &Planner3DMapSize::cbStatus, this);
    pub_map_ = nh_.advertise<costmap_cspace_msgs::CSpace3D>("costmap", 1, true);
    pub_map_update_ = nh_.advertise<costmap_cspace_msgs::CSpace3DUpdate>("costmap_update", 1);
  }

  void cbStatus(const planner_cspace_msgs::msg::PlannerStatus::ConstPtr& msg)
  {
    ++cnt_status_;
  }

  virtual void SetUp()
  {
    // Wait planner
    ASSERT_TRUE(waitStatus(rclcpp::Duration(5))) << "planner_3d is not up";
  }

  bool waitStatus(const rclcpp::Duration timeout)
  {
    const rclcpp::Time deadline = rclcpp::Time::now() + timeout;
    while (rclcpp::ok())
    {
      rclcpp::Duration(0.1).sleep();
      rclcpp::spin_some();

      if (cnt_status_ > 5)
        return true;

      if (deadline < rclcpp::Time::now())
        break;
    }
    return false;
  }

  costmap_cspace_msgs::CSpace3D generateCSpace3DMsg(
      const rclcpp::Time stamp, const size_t w, const size_t h, const size_t angle)
  {
    costmap_cspace_msgs::CSpace3D msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "msg";
    msg.info.width = w;
    msg.info.height = h;
    msg.info.angle = angle;
    msg.info.linear_resolution = 0.1;
    msg.info.angular_resolution = M_PI * 2 / angle;
    msg.info.origin.orientation.w = 1;
    msg.data.resize(msg.info.width * msg.info.height * msg.info.angle);
    for (auto& c : msg.data)
    {
      c = 100;
    }
    return msg;
  }

  costmap_cspace_msgs::CSpace3DUpdate generateCSpace3DUpdateMsg(
      const rclcpp::Time stamp,
      const size_t x, const size_t y, const size_t yaw,
      const size_t w, const size_t h, const size_t angle)
  {
    costmap_cspace_msgs::CSpace3DUpdate msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "msg";
    msg.x = x;
    msg.y = y;
    msg.yaw = yaw;
    msg.width = w;
    msg.height = h;
    msg.angle = angle;
    msg.data.resize(msg.width * msg.height * msg.angle);
    for (auto& c : msg.data)
    {
      c = 100;
    }
    return msg;
  }
};

TEST_F(Planner3DMapSize, OutOfRangeX)
{
  const rclcpp::Time now = rclcpp::Time::now();
  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  rclcpp::Duration(0.1).sleep();
  pub_map_update_->publish(generateCSpace3DUpdateMsg(now, 0, 0, 0, 0x81, 0x80, 4));
  rclcpp::Duration(0.5).sleep();
  rclcpp::spin_some();
  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(rclcpp::Duration(2)));
}

TEST_F(Planner3DMapSize, OutOfRangeY)
{
  const rclcpp::Time now = rclcpp::Time::now();
  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  rclcpp::Duration(0.1).sleep();
  pub_map_update_->publish(generateCSpace3DUpdateMsg(now, 0, 0, 0, 0x80, 0x81, 4));
  rclcpp::Duration(0.5).sleep();
  rclcpp::spin_some();
  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(rclcpp::Duration(2)));
}

TEST_F(Planner3DMapSize, OutOfRangeAngle)
{
  const rclcpp::Time now = rclcpp::Time::now();
  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  rclcpp::Duration(0.1).sleep();
  pub_map_update_->publish(generateCSpace3DUpdateMsg(now, 0, 0, 0, 0x80, 0x80, 8));
  rclcpp::Duration(0.5).sleep();
  rclcpp::spin_some();
  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(rclcpp::Duration(2)));
}

TEST_F(Planner3DMapSize, OutOfRangeAll)
{
  const rclcpp::Time now = rclcpp::Time::now();
  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  rclcpp::Duration(0.1).sleep();
  pub_map_update_->publish(generateCSpace3DUpdateMsg(now, 0, 0, 0, 0x81, 0x81, 8));
  rclcpp::Duration(0.5).sleep();
  rclcpp::spin_some();
  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(rclcpp::Duration(2)));
}

TEST_F(Planner3DMapSize, ZeroSizeUpdate)
{
  const rclcpp::Time now = rclcpp::Time::now();
  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  rclcpp::Duration(0.1).sleep();
  pub_map_update_->publish(generateCSpace3DUpdateMsg(now, 1, 1, 0, 0, 0, 0));
  rclcpp::Duration(0.5).sleep();
  rclcpp::spin_some();
  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(rclcpp::Duration(2)));
}

TEST_F(Planner3DMapSize, IllOrderedUpdate)
{
  const rclcpp::Time now = rclcpp::Time::now();
  const rclcpp::Time next = now + rclcpp::Duration(0.1);

  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  rclcpp::Duration(0.1).sleep();

  pub_map_update_->publish(generateCSpace3DUpdateMsg(next, 0, 0, 0, 0x81, 0x81, 8));
  rclcpp::Duration(0.1).sleep();

  pub_map_->publish(generateCSpace3DMsg(next, 0x81, 0x81, 8));
  rclcpp::Duration(0.5).sleep();

  rclcpp::spin_some();
  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(rclcpp::Duration(2)));
}

TEST_F(Planner3DMapSize, IllOrderedUpdateShrink)
{
  const rclcpp::Time now = rclcpp::Time::now();
  const rclcpp::Time next = now + rclcpp::Duration(0.1);

  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  rclcpp::Duration(0.1).sleep();

  pub_map_update_->publish(generateCSpace3DUpdateMsg(next, 0, 0, 0, 0x81, 0x81, 8));
  rclcpp::Duration(0.1).sleep();

  pub_map_->publish(generateCSpace3DMsg(next, 0x40, 0x40, 4));
  rclcpp::Duration(0.5).sleep();

  rclcpp::spin_some();
  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(rclcpp::Duration(2)));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv, "test_planner_cspace_map_size");

  return RUN_ALL_TESTS();
}
