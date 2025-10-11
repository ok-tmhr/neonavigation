/*
 * Copyright (c) 2014-2017, the neonavigation authors
 * Copyright (c) 2025, Tomohiro Oku
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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <nav2_map_server/map_io.hpp>
#include <yaml-cpp/yaml.h>

namespace map_organizer
{
class TieMapNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr pub_map_array_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pub_map_;

public:
  TieMapNode(const rclcpp::NodeOptions& options) : Node("tie_maps", options)
  {
    pub_map_array_ = this->create_publisher<map_organizer_msgs::msg::OccupancyGridArray>("maps", rclcpp::QoS(1).transient_local());

    map_organizer_msgs::msg::OccupancyGridArray maps;

    double height;
    std::string files_str = this->declare_parameter("map_files", "");
    std::string frame_id = this->declare_parameter("frame_id", "map");

    int i = 0;
    std::string file;
    std::stringstream ss(files_str);
    while (std::getline(ss, file, ','))
    {
      std::ifstream fin(file);
      if (fin.fail())
      {
        RCLCPP_ERROR(this->get_logger(), "Map_server could not open %s.", file.c_str());
        rclcpp::shutdown();
        return;
      }

      YAML::Node doc = YAML::Load(fin);
      try
      {
        height = doc["height"].as<double>();
      }
      catch(YAML::Exception& e)
      {
        height = 0.0;
      }

      nav_msgs::msg::OccupancyGrid map_resp;
      nav2_map_server::loadMapFromYaml(file, map_resp);
      map_resp.info.origin.position.z = height;
      map_resp.info.map_load_time = this->now();
      map_resp.header.frame_id = frame_id;
      map_resp.header.stamp = this->now();
      RCLCPP_INFO(this->get_logger(), "Read a %d X %d map @ %.3lf m/cell",
               map_resp.info.width,
               map_resp.info.height,
               map_resp.info.resolution);
      maps.maps.push_back(map_resp);
      pub_map_.push_back(this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "map" + std::to_string(i), rclcpp::QoS(1).transient_local()));
      pub_map_.back()->publish(map_resp);
      i++;
    }
    pub_map_array_->publish(maps);
  }
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_organizer::TieMapNode)