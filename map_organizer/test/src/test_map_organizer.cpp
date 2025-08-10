/*
 * Copyright (c) 2019, the neonavigation authors
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
#include <std_msgs/msg/int32.hpp>
#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <gtest/gtest.h>

void validateMap0(const nav_msgs::msg::OccupancyGrid& map, const double z)
{
  ASSERT_EQ("map_ground", map.header.frame_id);
  ASSERT_EQ(2u, map.info.width);
  ASSERT_EQ(4u, map.data.size());
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.y);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.z);
  ASSERT_FLOAT_EQ(1.0, map.info.origin.orientation.w);
  ASSERT_FLOAT_EQ(0.1, map.info.resolution);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.position.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.position.y);
  ASSERT_FLOAT_EQ(z, map.info.origin.position.z);
  ASSERT_EQ(100, map.data[0]);
  ASSERT_EQ(0, map.data[1]);
  ASSERT_EQ(100, map.data[2]);
  ASSERT_EQ(100, map.data[3]);
}
void validateMap1(const nav_msgs::msg::OccupancyGrid& map, const double z)
{
  ASSERT_EQ("map_ground", map.header.frame_id);
  ASSERT_EQ(2u, map.info.width);
  ASSERT_EQ(4u, map.data.size());
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.y);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.z);
  ASSERT_FLOAT_EQ(1.0, map.info.origin.orientation.w);
  ASSERT_FLOAT_EQ(0.2, map.info.resolution);
  ASSERT_FLOAT_EQ(0.1, map.info.origin.position.x);
  ASSERT_FLOAT_EQ(0.1, map.info.origin.position.y);
  ASSERT_FLOAT_EQ(z, map.info.origin.position.z);
  ASSERT_EQ(100, map.data[0]);
  ASSERT_EQ(0, map.data[1]);
  ASSERT_EQ(0, map.data[2]);
  ASSERT_EQ(100, map.data[3]);
}

TEST(MapOrganizer, MapArray)
{
  auto nh = rclcpp::Node::make_shared("map_array");

  map_organizer_msgs::msg::OccupancyGridArray::ConstPtr maps;
  const std::function<void(const map_organizer_msgs::msg::OccupancyGridArray::ConstPtr)>
      cb = [&maps](const map_organizer_msgs::msg::OccupancyGridArray::ConstPtr msg) -> void
  {
    maps = msg;
  };
  auto sub = nh->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>("maps", rclcpp::QoS(1).transient_local(), cb);

  rclcpp::Rate rate(10.0);
  for (int i = 0; i < 100 && rclcpp::ok(); ++i)
  {
    rate.sleep();
    rclcpp::spin_some(nh);
    if (maps)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(maps));
  ASSERT_EQ(2u, maps->maps.size());

  validateMap0(maps->maps[0], 1.0);
  validateMap1(maps->maps[1], 10.0);
}

TEST(MapOrganizer, Maps)
{
  auto nh = rclcpp::Node::make_shared("maps");

  nav_msgs::msg::OccupancyGrid::ConstPtr map[2];
  const std::function<void(const nav_msgs::msg::OccupancyGrid::ConstPtr, int)>
      cb = [&map](const nav_msgs::msg::OccupancyGrid::ConstPtr msg,
                  const int id) -> void
  {
    map[id] = msg;
  };
  using std::placeholders::_1;
  auto sub0 =
      nh->create_subscription<nav_msgs::msg::OccupancyGrid>("map0", rclcpp::QoS(1).transient_local(), std::function<void(const nav_msgs::msg::OccupancyGrid::ConstPtr&)>(std::bind(cb, _1, 0)));
  auto sub1 =
      nh->create_subscription<nav_msgs::msg::OccupancyGrid>("map1", rclcpp::QoS(1).transient_local(), std::function<void(const nav_msgs::msg::OccupancyGrid::ConstPtr&)>(std::bind(cb, _1, 1)));

  rclcpp::Rate rate(10.0);
  for (int i = 0; i < 100 && rclcpp::ok(); ++i)
  {
    rate.sleep();
    rclcpp::spin_some(nh);
    if (map[0] && map[1])
      break;
  }
  for (int i = 0; i < 2; ++i)
  {
    ASSERT_TRUE(static_cast<bool>(map[i]));
  }
  validateMap0(*(map[0]), 1.0);
  validateMap1(*(map[1]), 10.0);
}

TEST(MapOrganizer, SelectMap)
{
  auto nh = rclcpp::Node::make_shared("select_map");

  nav_msgs::msg::OccupancyGrid::ConstPtr map;
  const std::function<void(const nav_msgs::msg::OccupancyGrid::ConstPtr)>
      cb = [&map](const nav_msgs::msg::OccupancyGrid::ConstPtr msg) -> void
  {
    map = msg;
  };
  auto sub =
      nh->create_subscription<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local(), cb);
  auto pub = nh->create_publisher<std_msgs::msg::Int32>("floor", 1);

  rclcpp::Rate rate(10.0);
  for (int i = 0; i < 100 && rclcpp::ok(); ++i)
  {
    rate.sleep();
    rclcpp::spin_some(nh);
    if (pub->get_subscription_count() > 0 && map)
      break;
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);
  ASSERT_TRUE(static_cast<bool>(map));
  validateMap0(*map, 0.0);

  std_msgs::msg::Int32 floor;
  floor.data = 2;  // invalid floor must be ignored
  pub->publish(floor);

  map = nullptr;
  for (int i = 0; i < 10 && rclcpp::ok(); ++i)
  {
    rate.sleep();
    rclcpp::spin_some(nh);
    if (map)
      break;
  }
  ASSERT_FALSE(static_cast<bool>(map));

  floor.data = 1;
  pub->publish(floor);

  map = nullptr;
  for (int i = 0; i < 100 && rclcpp::ok(); ++i)
  {
    rate.sleep();
    rclcpp::spin_some(nh);
    if (map)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(map));
  validateMap1(*map, 0.0);
}

TEST(MapOrganizer, SavedMapArray)
{
  auto nh = rclcpp::Node::make_shared("saved_map_array");

  map_organizer_msgs::msg::OccupancyGridArray::ConstPtr maps;
  const std::function<void(const map_organizer_msgs::msg::OccupancyGridArray::ConstPtr)>
      cb = [&maps](const map_organizer_msgs::msg::OccupancyGridArray::ConstPtr msg) -> void
  {
    maps = msg;
  };
  auto sub = nh->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>("saved/maps", rclcpp::QoS(1).transient_local(), cb);

  rclcpp::Rate rate(10.0);
  for (int i = 0; i < 100 && rclcpp::ok(); ++i)
  {
    rate.sleep();
    rclcpp::spin_some(nh);
    if (maps)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(maps));
  ASSERT_EQ(2u, maps->maps.size());

  validateMap0(maps->maps[0], 1.0);
  validateMap1(maps->maps[1], 10.0);

  // clean temporary files
  auto pnh = rclcpp::Node::make_shared("test_map_organizer");
  std::string file_prefix;
  if (pnh->get_parameter("file_prefix", file_prefix))
  {
    ASSERT_EQ(0, system(std::string("rm -f " + file_prefix + "*").c_str()));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
