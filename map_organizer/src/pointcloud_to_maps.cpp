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

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map_organizer/pointcloud_to_maps_parameters.hpp>

namespace map_organizer
{
class PointcloudToMapsNode : public rclcpp::Node
{
private:
  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pub_maps_;
  rclcpp::Publisher<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr pub_map_array_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  std::shared_ptr<pointcloud_to_maps::ParamListener> param_listener_;

public:
  PointcloudToMapsNode(const rclcpp::NodeOptions& options) : Node("pointcloud_to_maps", options)
  {
    sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "mapcloud",
        rclcpp::QoS(1).transient_local(), [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg){ cbPoints(msg); });
    pub_map_array_ = this->create_publisher<map_organizer_msgs::msg::OccupancyGridArray>("maps", rclcpp::QoS(1).transient_local());
    param_listener_ = std::make_shared<pointcloud_to_maps::ParamListener>(get_node_parameters_interface());
  }
  void cbPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y"), it_z(*msg, "z");

    auto params = param_listener_->get_params();

    int min_points;

    int robot_height = static_cast<int>(std::round(params.robot_height / params.grid));
    int floor_height = static_cast<int>(std::round(params.floor_height / params.grid));
    int floor_tolerance = static_cast<int>(std::round(params.floor_tolerance / params.grid));

    std::unordered_map<int, int> hist;
    int x_min = std::numeric_limits<int>::max(), x_max = std::numeric_limits<int>::min();
    int y_min = std::numeric_limits<int>::max(), y_max = std::numeric_limits<int>::min();
    int h_min = std::numeric_limits<int>::max(), h_max = std::numeric_limits<int>::min();
    const float inv_grid = 1.f / static_cast<float>(params.grid);

    auto it_end = it_x.end();
    for (; it_x != it_end; ++it_x, ++it_y, ++it_z)
    {
      const int x = static_cast<int>(std::floor(*it_x * inv_grid));
      const int y = static_cast<int>(std::floor(*it_y * inv_grid));
      const int h = static_cast<int>(std::floor(*it_z * inv_grid));
      x_min = std::min(x_min, x);
      y_min = std::min(y_min, y);
      h_min = std::min(h_min, h);
      x_max = std::max(x_max, x);
      y_max = std::max(y_max, y);
      h_max = std::max(h_max, h);
      hist[h]++;
    }

    const auto max_height = h_max;
    const auto min_height = h_min;
    const auto H = max_height - min_height + 1;
    std::vector<float> floor_area(H, 0.f);
    std::vector<float> floor_runnable_area(H, 0.f);

    nav_msgs::msg::MapMetaData mmd;
    mmd.resolution = params.grid;
    mmd.origin.position.x = x_min * params.grid;
    mmd.origin.position.y = y_min * params.grid;
    mmd.origin.orientation.w = 1.0;
    mmd.width = x_max - x_min + 1;
    mmd.height = y_max - y_min + 1;
    RCLCPP_INFO(this->get_logger(), "width %d, height %d", mmd.width, mmd.height);
    std::vector<nav_msgs::msg::OccupancyGrid> maps;

    int hist_max = std::numeric_limits<int>::lowest();
    for (const auto& [h, points] : hist)
    {
      hist_max = std::max(hist_max, points);
    }

    min_points = hist_max * params.points_thresh_rate;

    it_x = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
    it_y = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y");
    it_z = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z");
    it_end = it_x.end();

    std::vector<std::vector<std::array<int, 3>>> buckets(H);

    for (; it_x != it_end; ++it_x, ++it_y, ++it_z)
    {
      const int x = static_cast<int>(std::floor(*it_x * inv_grid));
      const int y = static_cast<int>(std::floor(*it_y * inv_grid));
      const int h = static_cast<int>(std::floor(*it_z * inv_grid));
      buckets[h - min_height].push_back({x, y, h});
    }

    float floor_area_max = 0.f;
    double floor_runnable_area_max = 0;
    const auto cell_area = params.grid * params.grid;
    auto pack = [](int x, int y)->uint64_t { return uint64_t(uint32_t(x)) << 32 | uint64_t(uint32_t(y)); };
    std::vector<std::unordered_map<uint64_t, char>> floor(H);

    for (int i = 0; i < H; i++)
    {
      const int h = min_height + i;
      if (hist[h] <= min_points)
      {
        floor_area[i] = 0.f;
        continue;
      }

      auto& layer = floor[i];
      auto& pts = buckets[i];
      layer.reserve(pts.size());

      for (auto& [x, y, z] : pts)
      {
        auto key = pack(x, y);

        if (std::abs(h - z) <= floor_height)
        {
          layer.try_emplace(key, 0);
        }
        else if (h + floor_height + floor_tolerance < z && z <= h + robot_height)
        {
          layer.insert_or_assign(key, 1);
        }
      }

      int cnt = 0;
      for (const auto& [pt, occ] : layer)
      {
        if (occ == 0)
        {
          cnt++;
        }
      }

      floor_runnable_area[i] = cnt * cell_area;
      floor_area[i] = layer.size() * cell_area;
      floor_area_max = std::max(floor_area_max, floor_area[i]);
      floor_runnable_area_max = std::max(floor_runnable_area_max, static_cast<double>(floor_runnable_area[i]));
    }
    const double floor_area_filter = floor_runnable_area_max * params.floor_area_thresh_rate;
    int map_num = 0;
    auto is_peak = [&](int h) {
      const int i = h - min_height;
      return (h == min_height || floor_runnable_area[i - 1] <= floor_runnable_area[i]) &&
          (h == max_height || floor_runnable_area[i + 1] <= floor_runnable_area[i]);
    };

    for (int i = 0; i < H; i++)
    {
      const int h = i + min_height;
      if (hist[h] <= min_points)
      {
        continue;
      }

      if (!is_peak(h))
      {
        continue;
      }

      if (floor_runnable_area[i] <= floor_area_filter)
      {
        continue;
      }

      nav_msgs::msg::OccupancyGrid map;
      map.info = mmd;
      map.info.origin.position.z = h * params.grid;
      map.header = msg->header;
      map.data.assign(mmd.width * mmd.height, -1);
      auto unpack = [](uint64_t k)->std::pair<int, int>
      {
        int x = int(int32_t(k >> 32));
        int y = int(int32_t(uint32_t(k)));
        return {x, y};
      };

      for (const auto& [xy, occ] : floor[i])
      {
        auto [gx_raw, gy_raw] = unpack(xy);
        const int addr = (gx_raw - x_min) + (gy_raw - y_min) * mmd.width;
        if (occ == 0)
        {
          map.data[addr] = 0;
        }
        else if (occ == 1)
          map.data[addr] = 100;
      }

      maps.push_back(std::move(map));
      map_num++;
    }
    RCLCPP_INFO(this->get_logger(), "Floor candidates: %d", map_num);
    const double merge_threshold = params.grid * 1.5;
    for (size_t i = maps.size() - 1; i > 0; i--)
    {
      auto& cur = maps[i];
      auto& prev = maps[i - 1];

      const auto& z_cur = cur.info.origin.position.z;
      const auto& z_prev = prev.info.origin.position.z;

      if (std::abs(z_cur - z_prev) >= merge_threshold)
      {
        continue;
      }

      for (size_t j = 0; j < cur.data.size(); j++)
      {
        const auto c = cur.data[j];
        const auto p = prev.data[j];

        if (c != 0 && p == 0)
        {
          cur.data[j] = 0;
          prev.data[j] = -1;
        }
        else if (c == 0 && p == 0)
        {
          prev.data[j] = -1;
        }
      }

      auto cnt = [&](const decltype(cur) map){
        return std::count(map.data.begin(), map.data.end(), 0);
      };

      const int i_cur = int(z_cur * inv_grid) - min_height;
      const int i_prev = int(z_prev * inv_grid) - min_height;

      floor_runnable_area[i_cur] = cnt(cur) * cell_area;
      floor_runnable_area[i_prev] = cnt(prev) * cell_area;
    }

    for (int h = max_height; h >= min_height; h--)
    {
      const int i = h - min_height;
      double z = h * params.grid;
      std::string bar;
      int bar_len = (hist[h] * 16) / hist_max;
      bar.reserve(16);

      for (int j = 0; j <= 16; j++)
      {
        if (j <= bar_len)
          bar.push_back('#');
        else
          bar.push_back(' ');
      }
      if (floor_runnable_area[i] == 0.0)
        RCLCPP_INFO(this->get_logger(), "%6.2f %s  (%7d points)", z, bar.c_str(), hist[h]);
      else
        RCLCPP_INFO(this->get_logger(), "%6.2f %s  (%7d points, %5.2f m^2 of floor))", z, bar.c_str(), hist[h], floor_runnable_area[i]);
    }

    int num = -1;
    int floor_num = 0;
    map_organizer_msgs::msg::OccupancyGridArray map_array;
    for (auto& map : maps)
    {
      num++;
      int h = map.info.origin.position.z / params.grid;
      if (floor_runnable_area[h] < params.min_floor_area)
      {
        RCLCPP_WARN(this->get_logger(), "floor %d (%5.2fm^2), h = %0.2fm skipped",
                 floor_num, floor_runnable_area[h], map.info.origin.position.z);
        continue;
      }

      {
        const auto& src = map.data;

        const int& mW = mmd.width;
        const int& mH = mmd.height;
        const int R = 6;
        std::vector<int> dist(mW * mH, std::numeric_limits<int>::max());

        for (int i = 0; i < mW * mH; i++) {
          if (src[i] == 100)
            dist[i] = 0;
        }

        for (int y = 0; y < mH; y++) {
          for (int x = 0; x < mW; x++) {
            int i = x + y * mW;
            if (x > 0) {
              dist[i] = std::min(dist[i], dist[i - 1] + 1);
            }
            if (y > 0) {
              dist[i] = std::min(dist[i], dist[i - mW] + 1);
            }
          }
        }
        for (int y = mH - 1; y >= 0; y--) {
          for (int x = mW - 1; x >= 0; x--) {
            int i = x + y * mW;

            if (x + 1 < mW)
              dist[i] = std::min(dist[i], dist[i + 1] + 1);
            if (y + 1 < mH)
              dist[i] = std::min(dist[i], dist[i + mW] + 1);
          }
        }

        for (int i = 0; i < mW * mH; i++) {
          if (dist[i] > R) {
            map.data[i] = 0;
          }
        }
      }

      std::string name = "map" + std::to_string(floor_num);
      pub_maps_[name] = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/" + name, rclcpp::QoS(1).transient_local());
      pub_maps_[name]->publish(map);
      map_array.maps.push_back(map);
      RCLCPP_WARN(this->get_logger(), "floor %d (%5.2fm^2), h = %0.2fm",
               floor_num, floor_runnable_area[h], map.info.origin.position.z);
      floor_num++;
    }
    pub_map_array_->publish(map_array);
  }
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_organizer::PointcloudToMapsNode)
