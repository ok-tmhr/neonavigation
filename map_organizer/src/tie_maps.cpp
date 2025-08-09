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

#define HAVE_NEW_YAMLCPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
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

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class TieMapNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr pub_map_array_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pub_map_;

public:
  TieMapNode()
    : Node("tie_maps")
  {
    pub_map_array_ = this->create_publisher<map_organizer_msgs::msg::OccupancyGridArray>("maps", rclcpp::QoS(1).transient_local());

    map_organizer_msgs::msg::OccupancyGridArray maps;

    std::string files_str;
    std::string mapfname;
    double res;
    double origin[3], height;
    int negate;
    double occ_th, free_th;
    nav2_map_server::MapMode mode;
    std::string frame_id;
    files_str = this->declare_parameter("map_files", std::string(""));
    frame_id = this->declare_parameter("frame_id", std::string("map"));

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
#ifdef HAVE_NEW_YAMLCPP
      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);
#else
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
#endif
      try
      {
        doc["resolution"] >> res;
      }
      catch (YAML::InvalidScalar& e)
      {
        RCLCPP_ERROR(this->get_logger(), "The map does not contain a resolution tag or it is invalid: %s", e.what());
        rclcpp::shutdown();
        return;
      }
      try
      {
        doc["negate"] >> negate;
      }
      catch (YAML::InvalidScalar& e)
      {
        RCLCPP_ERROR(this->get_logger(), "The map does not contain a negate tag or it is invalid: %s", e.what());
        rclcpp::shutdown();
        return;
      }
      try
      {
        doc["occupied_thresh"] >> occ_th;
      }
      catch (YAML::InvalidScalar& e)
      {
        RCLCPP_ERROR(this->get_logger(), "The map does not contain an occupied_thresh tag or it is invalid: %s", e.what());
        rclcpp::shutdown();
        return;
      }
      try
      {
        doc["free_thresh"] >> free_th;
      }
      catch (YAML::InvalidScalar& e)
      {
        RCLCPP_ERROR(this->get_logger(), "The map does not contain a free_thresh tag or it is invalid: %s", e.what());
        rclcpp::shutdown();
        return;
      }
      try
      {
        std::string modeS = "";
        doc["mode"] >> modeS;

        if (modeS == "trinary")
          mode = nav2_map_server::MapMode::Trinary;
        else if (modeS == "scale")
          mode = nav2_map_server::MapMode::Scale;
        else if (modeS == "raw")
          mode = nav2_map_server::MapMode::Raw;
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Invalid mode tag \"%s\".", modeS.c_str());
          exit(-1);
        }
      }
      catch (YAML::Exception& e)
      {
        RCLCPP_DEBUG(this->get_logger(), "The map does not contain a mode tag or it is invalid... assuming trinary: %s", e.what());
        mode = nav2_map_server::MapMode::Trinary;
      }
      try
      {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
      }
      catch (YAML::InvalidScalar& e)
      {
        RCLCPP_ERROR(this->get_logger(), "The map does not contain an origin tag or it is invalid: %s", e.what());
        rclcpp::shutdown();
        return;
      }
      try
      {
        doc["height"] >> height;
      }
      catch (YAML::Exception& e)
      {
        height = 0;
      }
      try
      {
        doc["image"] >> mapfname;
        // TODO(at-wat): make this path-handling more robust
        if (mapfname.size() == 0)
        {
          RCLCPP_ERROR(this->get_logger(), "The image tag cannot be an empty string.");
          rclcpp::shutdown();
          return;
        }
        if (mapfname[0] != '/')
        {
          // dirname can modify what you pass it
          char* fname_copy = strdup(file.c_str());
          mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
          free(fname_copy);
        }
      }
      catch (YAML::InvalidScalar& e)
      {
        RCLCPP_ERROR(this->get_logger(), "The map does not contain an image tag or it is invalid: e.what()");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Loading map from image \"%s\"", mapfname.c_str());

      nav_msgs::msg::OccupancyGrid map_resp;
      nav2_map_server::LoadParameters load_parameters{
        .image_file_name = mapfname,
        .resolution = res,
        .origin = std::vector<double>(origin, origin + 3),
        .free_thresh = free_th,
        .occupied_thresh = occ_th,
        .mode = mode,
        .negate = negate
      };
      nav2_map_server::loadMapFromFile(load_parameters, map_resp);
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto tmn = std::make_shared<TieMapNode>();
  rclcpp::spin(tmn);

  return 0;
}
