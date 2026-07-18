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

#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>

#include "map_organizer/save_maps_parameter.hpp"

namespace fs = std::filesystem;

/**
 * @brief Map generation node.
 */
class MapGeneratorNode : public rclcpp::Node
{
protected:
  std::string mapname_;
  rclcpp::Subscription<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr map_sub_;
  bool saved_map_;
  std::shared_ptr<save_maps::ParamListener> param_listener_;

public:
  explicit MapGeneratorNode(const rclcpp::NodeOptions& options=rclcpp::NodeOptions()) : Node("save_maps", options)
    , saved_map_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for the map");
    map_sub_ = this->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>("maps", rclcpp::QoS(1).transient_local(),
      [this](const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr msg){ mapsCallback(msg); });
    param_listener_ = std::make_shared<save_maps::ParamListener>(this->get_node_parameters_interface());
    mapname_ = param_listener_->get_params().map_name;
  }

  bool done() const
  {
    return saved_map_;
  }
  void mapsCallback(const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr maps)
  {
    int i = 0;
    for (auto& map : maps->maps)
    {
      mapCallback(&map, i);
      i++;
    }
    saved_map_ = true;
    rclcpp::shutdown();
  }
  void mapCallback(const nav_msgs::msg::OccupancyGrid* map, const int floor)
  {
    RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix",
             map->info.width,
             map->info.height,
             map->info.resolution);

    fs::path mapdatafile(mapname_ + std::to_string(floor) + ".pgm");
    RCLCPP_INFO(this->get_logger(), "Writing map occupancy data to %s", mapdatafile.c_str());

    std::ofstream out(mapdatafile, std::ios::binary);
    if (!out)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't save map file to %s", mapdatafile.c_str());
      return;
    }

    out << "P5\n# CREATOR: Map_generator.cpp"
        << std::fixed << std::setprecision(3)
        << map->info.resolution << " m/pix\n"
        << map->info.width << " "
        << map->info.height << "\n255\n";

    std::vector<u_char> buffer(map->info.width * map->info.height);
    for (unsigned int y = 0; y < map->info.height; y++)
    {
      for (unsigned int x = 0; x < map->info.width; x++)
      {
        unsigned int i = x + (map->info.height - y - 1) * map->info.width;
        switch (map->data[i])
        {
          case 0: // occ [0,0.1)
            buffer[x + y * map->info.width] = 254;
            break;
          case 100: // occ (0.65,1]
            buffer[x + y * map->info.width] = 0;
            break;
          default: // occ [0.1,0.65]
            buffer[x + y * map->info.width] = 205;
            break;
        }
      }
    }

    out.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    out.close();

    fs::path mapmetadatafile(mapname_ + std::to_string(floor) + ".yaml");
    RCLCPP_INFO(this->get_logger(), "Writing map occupancy data to %s", mapmetadatafile.c_str());
    std::ofstream yaml(mapmetadatafile);

    double yaw = tf2::getYaw(map->info.origin.orientation);

    const auto& position = map->info.origin.position;
    yaml << "image: " << mapdatafile.filename().string()
         << "\nresolution: " << map->info.resolution
         << "\norigin: [" << position.x << ", " << position.y << ", " << yaw << "]"
         << "\nheight: " << position.z
         << "\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n";

    RCLCPP_INFO(this->get_logger(), "Done\n");
  }
};

#define USAGE "Usage: \n"        \
              "  map_saver -h\n" \
              "  map_saver [-f <mapname>] [ROS arguments]"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::string mapname = "";

  for (size_t i = 1; i < args.size(); i++)
  {
    if (args[i] == "-h")
    {
      std::cout << USAGE << std::endl;
      return 0;
    }
    else if (args[i] == "-f" && ++i < args.size())
    {
      mapname = argv[i];
    }
    else
    {
      std::cout << USAGE << std::endl;;
      return 1;
    }
  }

  auto mg = std::make_shared<MapGeneratorNode>(rclcpp::NodeOptions());
  if (!mapname.empty()){
    mg->set_parameter({"map_name", mapname});
  }
  // while (!mg->done() && rclcpp::ok())
  rclcpp::spin(mg);

  return 0;
}
