/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2016, the neonavigation authors
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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
#include <nav_msgs/srv/get_map.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <fmt/format.h>
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>

namespace fs = std::filesystem;

void writeMapMeta(const fs::path& yaml_path, const fs::path& image_path,
                  const nav_msgs::msg::OccupancyGrid& map)
{
  YAML::Emitter out;

  const auto& pos = map.info.origin.position;
  const auto yaw = tf2::getYaw(map.info.origin.orientation);

  out << YAML::BeginMap;

  out << YAML::Key << "image" << YAML::Value << image_path.filename().string();
  out << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
  out << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq
      << pos.x << pos.y << yaw << YAML::EndSeq;
  out << YAML::Key << "height" << YAML::Value << pos.z;
  out << YAML::Key << "negate" << YAML::Value << 0;
  out << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
  out << YAML::Key << "free_thresh" << YAML::Value << 0.196;

  out << YAML::EndMap;

  std::ofstream fout(yaml_path);
  fout << out.c_str();
}

bool writeMap(const fs::path& filepath, const nav_msgs::msg::OccupancyGrid& map)
{
  const auto& width = map.info.width;
  const auto& height = map.info.height;

  std::ofstream out(filepath, std::ios::binary);
  if (!out) {
    return false;
  }

  out << "P5\n# CREATOR: save_maps.cpp" << std::fixed << std::setprecision(3)
      << map.info.resolution << " m/pix\n"
      << width << " " << height << "\n255\n";

  std::vector<std::uint8_t> buffer(width * height);
  for (unsigned int y = 0; y < height; y++) {
    for (unsigned int x = 0; x < width; x++) {
      const unsigned int src = x + (height - y - 1) * width;
      const unsigned int dst = x + y * width;

      switch (map.data[src]) {
      case 0: // occ [0,0.1)
        buffer[dst] = 254;
        break;
      case 100: // occ (0.65,1]
        buffer[dst] = 0;
        break;
      default: // occ [0.1,0.65]
        buffer[dst] = 205;
        break;
      }
    }
  }

  out.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
  return true;
}

/**
 * @brief Map generation node.
 */
class SaveMapsNode : public rclcpp::Node
{
protected:
  std::string mapname_;
  rclcpp::Subscription<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  explicit SaveMapsNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("save_maps", options)
  {
    mapname_ = this->declare_parameter("map_name", "map");
    RCLCPP_INFO(this->get_logger(), "Waiting for the map");
    map_sub_ = this->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>("maps", rclcpp::QoS(1).transient_local(),
      [this](const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr msg){ mapsCallback(msg); });
  }

  void mapsCallback(const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr maps)
  {
    int floor = 0;
    for (const auto& map : maps->maps)
    {
      RCLCPP_INFO(this->get_logger(), "Received a %u X %u map @ %.3f m/pix",
                  map.info.width, map.info.height, map.info.resolution);

      const fs::path mapdatafile = fmt::format("{}{}.pgm", mapname_, floor);
      RCLCPP_INFO(this->get_logger(), "Writing map occupancy data to %s",
                  mapdatafile.c_str());

      if (!writeMap(mapdatafile, map)) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't save map file to %s",
                     mapdatafile.c_str());
        return;
      }

      const fs::path mapmetadatafile =
          fmt::format("{}{}.yaml", mapname_, floor);
      RCLCPP_INFO(this->get_logger(), "Writing map occupancy data to %s",
                  mapmetadatafile.c_str());
      writeMapMeta(mapmetadatafile, mapdatafile, map);

      RCLCPP_INFO(this->get_logger(), "Done");
      floor++;
    }
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [](){ rclcpp::shutdown(); });

  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto mg = std::make_shared<SaveMapsNode>();

  while (rclcpp::ok())
  {
    rclcpp::spin_some(mg);
  }

  return 0;
}
