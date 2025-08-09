/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>


rclcpp::Node::SharedPtr node;
map_organizer_msgs::msg::OccupancyGridArray maps;
std::vector<nav_msgs::msg::MapMetaData> orig_mapinfos;
int floor_cur = 0;

void cbMaps(const map_organizer_msgs::msg::OccupancyGridArray::Ptr msg)
{
  RCLCPP_INFO(node->get_logger(), "Map array received");
  maps = *msg;
  orig_mapinfos.clear();
  for (auto& map : maps.maps)
  {
    orig_mapinfos.push_back(map.info);
    map.info.origin.position.z = 0.0;
  }
}
void cbFloor(const std_msgs::msg::Int32::Ptr msg)
{
  floor_cur = msg->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("select_map");

  auto subMaps = node->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>(
      "maps",
      1, cbMaps);
  auto subFloor = node->create_subscription<std_msgs::msg::Int32>(
      "floor",
      1, cbFloor);
  auto pubMap = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map",
      rclcpp::QoS(1).transient_local());

  auto tfb = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped trans;
  trans.header.frame_id = "map_ground";
  trans.child_frame_id = "map";
  trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 0.0));

  rclcpp::Rate wait(10);
  int floor_prev = -1;
  while (rclcpp::ok())
  {
    wait.sleep();
    rclcpp::spin_some(node);

    if (maps.maps.size() == 0)
      continue;

    if (floor_cur != floor_prev)
    {
      if (floor_cur >= 0 && floor_cur < static_cast<int>(maps.maps.size()))
      {
        pubMap->publish(maps.maps[floor_cur]);
        trans.transform.translation.z = orig_mapinfos[floor_cur].origin.position.z;
      }
      else
      {
        RCLCPP_INFO(node->get_logger(), "Floor out of range");
      }
      floor_prev = floor_cur;
    }
    trans.header.stamp = node->now() + rclcpp::Duration::from_seconds(0.15);
    tfb->sendTransform(trans);
  }

  return 0;
}
