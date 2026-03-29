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

#include <map_organizer/pointcloud_to_maps_parameters.hpp>

namespace map_organizer
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct GridRange
{
  int x_min, x_max, y_min, y_max, z_min, z_max;
  GridRange(int x0, int x1, int y0, int y1, int z0, int z1)
  : x_min(x0), x_max(x1), y_min(y0), y_max(y1), z_min(z0), z_max(z1)
  {}

  int x_span() const { return x_max - x_min + 1; }
  int y_span() const { return y_max - y_min + 1; }
  int z_span() const { return z_max - z_min + 1; }

};

enum OCCUPANCY {
  UNKNOWN = -1,
  FREE = 0,
  OCCUPIED = 100
};

inline bool has_field(const PointCloud2& cloud, const std::string& field_name)
{
  for (const auto& field : cloud.fields)
  {
    if (field.name == field_name)
    {
      return true;
    }
  }
  return false;
}

template<typename Func>
inline void processPointCloudXYZ(const PointCloud2& msg, const float inv_grid, Func func)
{
  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x"), it_y(msg, "y"), it_z(msg, "z");
  const auto it_end = it_x.end();
  for (; it_x != it_end; ++it_x, ++it_y, ++it_z)
  {
    if (std::isfinite(*it_x) && std::isfinite(*it_y) && std::isfinite(*it_z))
    {
      const int x = static_cast<int>(std::floor(*it_x * inv_grid));
      const int y = static_cast<int>(std::floor(*it_y * inv_grid));
      const int z = static_cast<int>(std::floor(*it_z * inv_grid));
      func(x, y, z);
    }
  }
}

template<typename Func>
inline void processPointCloudZ(const PointCloud2& msg, const float inv_grid, Func func)
{
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
  const auto it_end = it_z.end();
  for (; it_z != it_end; ++it_z)
  {
    if (std::isfinite(*it_z))
    {
      const int z = static_cast<int>(std::floor(*it_z * inv_grid));
      func(z);
    }
  }
}

class PointcloudToMapsNode : public rclcpp::Node
{
private:
  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pub_maps_;
  rclcpp::Publisher<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr pub_map_array_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_points_;
  std::shared_ptr<pointcloud_to_maps::ParamListener> param_listener_;
  pointcloud_to_maps::Params params_;
  int robot_height_, floor_height_, floor_tolerance_;

public:
  PointcloudToMapsNode(const rclcpp::NodeOptions& options) : Node("pointcloud_to_maps", options)
  {
    sub_points_ = this->create_subscription<PointCloud2>(
        "mapcloud",
        rclcpp::QoS(1).transient_local(), [this](const PointCloud2::SharedPtr msg){ cbPoints(msg); });
    pub_map_array_ = this->create_publisher<map_organizer_msgs::msg::OccupancyGridArray>("maps", rclcpp::QoS(1).transient_local());
    param_listener_ = std::make_shared<pointcloud_to_maps::ParamListener>(get_node_parameters_interface());
    cbParam(param_listener_->get_params());
  }

  void cbParam(const pointcloud_to_maps::Params& params)
  {
    params_ = params;
    robot_height_ = static_cast<int>(std::floor(params.robot_height / params.grid));
    floor_height_ = static_cast<int>(std::floor(params.floor_height / params.grid));
    floor_tolerance_ = static_cast<int>(std::floor(params.floor_tolerance / params.grid));
  }

  void cbPoints(const PointCloud2::ConstSharedPtr msg)
  {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0){
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Empty point cloud");
      return;
    }

    if (!has_field(*msg, "x") || !has_field(*msg, "y") || !has_field(*msg, "z"))
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Point cloud missing required fields (x, y, z)");
      return;
    }

    const auto inv_grid = 1.f / static_cast<float>(params_.grid);

    const auto range = [msg, inv_grid](){
      int x_min = std::numeric_limits<int>::max(), x_max = std::numeric_limits<int>::min();
      int y_min = std::numeric_limits<int>::max(), y_max = std::numeric_limits<int>::min();
      int h_min = std::numeric_limits<int>::max(), h_max = std::numeric_limits<int>::min();

      processPointCloudXYZ(*msg, inv_grid, [&](int x, int y, int z) {
        x_min = std::min(x_min, x);
        y_min = std::min(y_min, y);
        h_min = std::min(h_min, z);
        x_max = std::max(x_max, x);
        y_max = std::max(y_max, y);
        h_max = std::max(h_max, z);
      });

      return GridRange(x_min, x_max, y_min, y_max, h_min, h_max);
    }();

    const auto H = range.z_span();
    std::vector<int> hist(H, 0);
    processPointCloudZ(*msg, inv_grid, [&hist, &range](int z){ hist[z - range.z_min]++; });
    std::vector<int> floor_runnable_area(H, 0);

    nav_msgs::msg::MapMetaData mmd;
    mmd.resolution = static_cast<float>(params_.grid);
    mmd.origin.position.x = range.x_min * params_.grid;
    mmd.origin.position.y = range.y_min * params_.grid;
    mmd.origin.orientation.w = 1.0;
    mmd.width = static_cast<uint32_t>(range.x_span());
    mmd.height = static_cast<uint32_t>(range.y_span());
    RCLCPP_DEBUG(this->get_logger(), "width %d, height %d", mmd.width, mmd.height);
    std::vector<nav_msgs::msg::OccupancyGrid> maps;

    const auto hist_max = *std::max_element(hist.begin(), hist.end());
    const auto min_points = static_cast<int>(hist_max * params_.points_thresh_rate);
    const auto cell_area = params_.grid * params_.grid;
    std::vector<std::vector<int8_t>> floor(H, std::vector<int8_t>(mmd.width * mmd.height, OCCUPANCY::UNKNOWN));
    std::vector<std::vector<int>> active_indices(H);
    std::vector<uint8_t> visited(mmd.width * mmd.height, 0);
    std::vector<int> touched;
    touched.reserve(1024);

    processPointCloudXYZ(*msg, inv_grid, [&](int x, int y, int z) {
      const auto index = (x - range.x_min) + range.x_span() * (y - range.y_min);
      const auto h = z - range.z_min;
      if (floor[h][index] == OCCUPANCY::UNKNOWN)
      {
        floor[h][index] = OCCUPANCY::FREE;
        active_indices[h].push_back(index);
        floor_runnable_area[h]++;
      }
    });

    for (auto h = 0; h < H; h++)
    {
      if (static_cast<int>(active_indices[h].size()) < min_points)
      {
        continue;
      }
      touched.clear();

      const auto i_begin = std::max(0, h - floor_height_);
      const auto i_end = std::min(H - 1, h + floor_height_);
      for (auto i = i_begin; i <= i_end; i++)
      {
          for (const auto& index : active_indices[i])
          {
              if (!visited[index])
              {
                  floor[h][index] = OCCUPANCY::FREE;
                  visited[index] = 1;
                  touched.push_back(index);
                  floor_runnable_area[h]++;
              }
          }
      }

      const auto i_begin2 = std::max(0, h - robot_height_);
      const auto i_end2 = std::min(H - 1, h - floor_height_ - floor_tolerance_);
      for (int i = i_begin2; i <= i_end2; i++)
      {
          for (const auto& index : active_indices[i])
          {
            auto& occ = floor[h][index];
            if (occ != OCCUPANCY::OCCUPIED)
            {
              if (occ == OCCUPANCY::FREE)
              {
                floor_runnable_area[h]--;
              }
              occ = OCCUPANCY::OCCUPIED;
            }
          }
      }

      for (const auto& i : touched)
      {
          visited[i] = 0;
      }
    }

    const auto floor_area_filter = static_cast<int>(*std::max_element(floor_runnable_area.begin(), floor_runnable_area.end()) * params_.floor_area_thresh_rate);
    auto is_peak = [H, &floor_runnable_area](const int h) {
      return (h == 0 || floor_runnable_area[h - 1] <= floor_runnable_area[h]) &&
          (h == H - 1 || floor_runnable_area[h + 1] <= floor_runnable_area[h]);
    };

    for (int h = 0; h < H; h++)
    {
      if (hist[h] > min_points && is_peak(h) && floor_runnable_area[h] > floor_area_filter)
      {
        nav_msgs::msg::OccupancyGrid map;
        map.info = mmd;
        map.info.origin.position.z = (h + range.z_min) * params_.grid;
        map.header = msg->header;
        map.data = std::move(floor[h]);
        maps.push_back(std::move(map));
      }
    }
    RCLCPP_DEBUG(this->get_logger(), "Floor candidates: %ld", maps.size());
    if (maps.empty())
    {
      return;
    }
    const auto merge_threshold = params_.grid * 1.5;
    for (auto i = maps.size() - 1; i > 0; i--)
    {
      auto& cur = maps[i];
      auto& prev = maps[i - 1];

      const auto& z_cur = cur.info.origin.position.z;
      const auto& z_prev = prev.info.origin.position.z;

      if (std::abs(z_cur - z_prev) >= merge_threshold)
      {
        continue;
      }

      for (auto j = 0UL; j < cur.data.size(); j++)
      {
        const auto c = cur.data[j];
        const auto p = prev.data[j];

        if (p == FREE)
        {
          prev.data[j] = UNKNOWN;
          if (c != FREE)
          {
            cur.data[j] = FREE;
          }
        }
      }

      auto cnt = [](const nav_msgs::msg::OccupancyGrid& map){
        return static_cast<int>(std::count(map.data.begin(), map.data.end(), FREE));
      };

      const auto i_cur = static_cast<int>(z_cur / params_.grid) - range.z_min;
      const auto i_prev = static_cast<int>(z_prev / params_.grid) - range.z_min;

      floor_runnable_area[i_cur] = cnt(cur);
      floor_runnable_area[i_prev] = cnt(prev);
    }

    if (rcutils_logging_get_logger_effective_level(this->get_logger().get_name()) <= RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG)
    {
      for (auto h = H - 1; h >= 0; h--)
      {
        std::string bar;
        const auto bar_len = (hist[h] * 16) / hist_max;
        bar.reserve(16);
        for (auto j = 0; j <= 16; j++)
        {
          bar.push_back(j <= bar_len ? '#' : ' ');
        }
        const auto z = (h + range.z_min) * params_.grid;
        if (floor_runnable_area[h] == 0)
        RCLCPP_DEBUG(this->get_logger(), "%6.2f %s  (%7d points)", z, bar.c_str(), hist[h]);
        else
        RCLCPP_DEBUG(this->get_logger(), "%6.2f %s  (%7d points, %5.2f m^2 of floor))", z, bar.c_str(), hist[h], floor_runnable_area[h] * cell_area);
      }
    }

    int floor_num = 0;
    map_organizer_msgs::msg::OccupancyGridArray map_array;
    pub_maps_.clear();
    for (auto&& map : maps)
    {
      const auto h = static_cast<int>(map.info.origin.position.z / params_.grid) - range.z_min;
      if (floor_runnable_area[h] * cell_area < params_.min_floor_area)
      {
        RCLCPP_WARN(this->get_logger(), "floor %d (%5.2fm^2), h = %0.2fm skipped",
                 floor_num, floor_runnable_area[h] * cell_area, map.info.origin.position.z);
        continue;
      }

      {
        const auto& src = map.data;

        const int& mW = map.info.width;
        const int& mH = map.info.height;
        const int R = 6;
        std::vector<int> dist(mW * mH, std::numeric_limits<int>::max());

        for (auto i = 0; i < mW * mH; i++) {
          if (src[i] == OCCUPIED)
            dist[i] = FREE;
        }

        for (int y = 0; y < mH; y++) {
          for (int x = 0; x < mW; x++) {
            const auto i = x + y * mW;
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
            map.data[i] = FREE;
          }
        }
      }

      const std::string name = "map" + std::to_string(floor_num);
      pub_maps_[name] = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/" + name, rclcpp::QoS(1).transient_local());
      pub_maps_[name]->publish(map);
      map_array.maps.push_back(map);
      RCLCPP_WARN(this->get_logger(), "floor %d (%5.2fm^2), h = %0.2fm",
               floor_num, floor_runnable_area[h] * cell_area, map.info.origin.position.z);
      floor_num++;
    }
    pub_map_array_->publish(map_array);
  }
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_organizer::PointcloudToMapsNode)
