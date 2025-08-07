/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2016, the neonavigation authors
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
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <cstdio>
#include <string>

#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>

/**
 * @brief Map generation node.
 */
class MapGeneratorNode : public rclcpp::Node
{
protected:
  std::string mapname_;
  rclcpp::Subscription<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr map_sub_;
  bool saved_map_;

public:
  explicit MapGeneratorNode(const std::string& mapname)
    : Node("save_maps")
    , mapname_(mapname)
    , saved_map_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for the map");
    map_sub_ = this->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>("maps", 1, std::bind(&MapGeneratorNode::mapsCallback, this, std::placeholders::_1));
  }

  bool done() const
  {
    return saved_map_;
  }
  void mapsCallback(const map_organizer_msgs::msg::OccupancyGridArray::ConstPtr& maps)
  {
    int i = 0;
    for (auto& map : maps->maps)
    {
      mapCallback(&map, i);
      i++;
    }
    saved_map_ = true;
  }
  void mapCallback(const nav_msgs::msg::OccupancyGrid* map, const int floor)
  {
    RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix",
             map->info.width,
             map->info.height,
             map->info.resolution);

    std::string mapdatafile = mapname_ + std::to_string(floor) + ".pgm";
    RCLCPP_INFO(this->get_logger(), "Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't save map file to %s", mapdatafile.c_str());
      return;
    }

    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    for (unsigned int y = 0; y < map->info.height; y++)
    {
      for (unsigned int x = 0; x < map->info.width; x++)
      {
        unsigned int i = x + (map->info.height - y - 1) * map->info.width;
        if (map->data[i] == 0)
        {  // occ [0,0.1)
          fputc(254, out);
        }
        else if (map->data[i] == +100)
        {  // occ (0.65,1]
          fputc(000, out);
        }
        else
        {  // occ [0.1,0.65]
          fputc(205, out);
        }
      }
    }

    fclose(out);

    std::string mapmetadatafile = mapname_ + std::to_string(floor) + ".yaml";
    RCLCPP_INFO(this->get_logger(), "Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

    geometry_msgs::msg::Quaternion orientation = map->info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\n"
                  "origin: [%f, %f, %f]\nheight: %f\n"
                  "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), map->info.resolution,
            map->info.origin.position.x, map->info.origin.position.y, yaw, map->info.origin.position.z);

    fclose(yaml);

    RCLCPP_INFO(this->get_logger(), "Done\n");
  }
};

#define USAGE "Usage: \n"        \
              "  map_saver -h\n" \
              "  map_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::string mapname = "map";

  for (int i = 1; i < argc; i++)
  {
    if (!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if (!strcmp(argv[i], "-f"))
    {
      if (++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  auto mg = std::make_shared<MapGeneratorNode>(mapname);

  while (!mg->done() && rclcpp::ok())
    rclcpp::spin_some(mg);

  return 0;
}
