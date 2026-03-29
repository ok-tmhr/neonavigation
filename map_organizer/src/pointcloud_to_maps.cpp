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
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map_organizer_msgs/msg/occupancy_grid_array.hpp>

#include <map_organizer/pointcloud_to_maps_parameters.hpp>

namespace map_organizer
{

constexpr int THROTTLE_MS = 3000;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct GridRange
{
  int x_min, x_max, y_min, y_max, z_min, z_max;

  GridRange(int x0, int x1, int y0, int y1, int z0, int z1)
  : x_min(x0), x_max(x1), y_min(y0), y_max(y1), z_min(z0), z_max(z1)
  {}

  bool valid() const
  {
    return x_min <= x_max && y_min <= y_max && z_min <= z_max;
  }

  int x_span() const { return x_max - x_min + 1; }
  int y_span() const { return y_max - y_min + 1; }
  int z_span() const { return z_max - z_min + 1; }
  int xy_span() const { return x_span() * y_span(); }

};

enum OCCUPANCY {
  UNKNOWN = -1,
  FREE = 0,
  OCCUPIED = 100
};

inline bool has_field(const PointCloud2& cloud, const std::string& field_name)
{
    return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                       [&](const sensor_msgs::msg::PointField& f) { return f.name == field_name; });
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
  std::vector<int> floor_runnable_area_;
  std::vector<int8_t> floor_;
  std::vector<uint8_t> visited_;
  std::vector<int> touched_;
  std::vector<int> hist_;
  std::vector<std::vector<int>> active_indices_;

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
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_MS, "Empty point cloud");
      return;
    }

    if (!has_field(*msg, "x") || !has_field(*msg, "y") || !has_field(*msg, "z"))
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_MS, "Point cloud missing required fields (x, y, z)");
      return;
    }

    const auto inv_grid = 1.f / static_cast<float>(params_.grid);

    const auto range = [&](){
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

    if (!range.valid())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_MS, "No valid points found in the point cloud");
      return;
    }

    const auto H = range.z_span();

    if (H == 0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_MS, "No valid height data found in the point cloud");
      return;
    }

    hist_.assign(H, 0);
    processPointCloudZ(*msg, inv_grid, [this, &range](int z){ hist_[z - range.z_min]++; });
    floor_runnable_area_.assign(H, 0);

    nav_msgs::msg::MapMetaData mmd;
    mmd.resolution = static_cast<float>(params_.grid);
    mmd.origin.position.x = range.x_min * params_.grid;
    mmd.origin.position.y = range.y_min * params_.grid;
    mmd.origin.orientation.w = 1.0;
    mmd.width = static_cast<uint32_t>(range.x_span());
    mmd.height = static_cast<uint32_t>(range.y_span());
    RCLCPP_DEBUG(this->get_logger(), "width %d, height %d", mmd.width, mmd.height);
    std::vector<nav_msgs::msg::OccupancyGrid> maps;

    const auto hist_max = *std::max_element(hist_.begin(), hist_.end());
    const auto min_points = static_cast<int>(hist_max * params_.points_thresh_rate);
    const auto cell_area = params_.grid * params_.grid;

    const auto map_area = range.xy_span();
    floor_.assign(H * map_area, OCCUPANCY::UNKNOWN);
    visited_.assign(map_area, 0);
    active_indices_.assign(H, {});
    touched_.reserve(map_area);

    processPointCloudXYZ(*msg, inv_grid, [&](int x, int y, int z) {
      const auto index = (x - range.x_min) + range.x_span() * (y - range.y_min);
      const auto h = z - range.z_min;
      if (floor_[h * map_area + index] == OCCUPANCY::UNKNOWN)
      {
        floor_[h * map_area + index] = OCCUPANCY::FREE;
        active_indices_[h].push_back(index);
        floor_runnable_area_[h]++;
      }
    });

    for (auto h = 0; h < H; h++)
    {
      if (static_cast<int>(active_indices_[h].size()) < min_points)
      {
        continue;
      }
      touched_.clear();

      const auto i_begin = std::max(0, h - floor_height_);
      const auto i_end = std::min(H - 1, h + floor_height_);
      for (auto i = i_begin; i <= i_end; i++)
      {
          for (const auto& index : active_indices_[i])
          {
              if (!visited_[index])
              {
                  floor_[h * map_area + index] = OCCUPANCY::FREE;
                  visited_[index] = 1;
                  touched_.push_back(index);
                  floor_runnable_area_[h]++;
              }
          }
      }

      const auto i_begin2 = std::max(0, h - robot_height_);
      const auto i_end2 = std::min(H - 1, h - floor_height_ - floor_tolerance_);
      for (int i = i_begin2; i <= i_end2; i++)
      {
          for (const auto& index : active_indices_[i])
          {
            auto& occ = floor_[h * map_area + index];
            if (occ != OCCUPANCY::OCCUPIED)
            {
              if (occ == OCCUPANCY::FREE)
              {
                floor_runnable_area_[h]--;
              }
              occ = OCCUPANCY::OCCUPIED;
            }
          }
      }

      for (const auto& i : touched_)
      {
          visited_[i] = 0;
      }

      floor_runnable_area_[h] = static_cast<int>(
          std::count(floor_.begin() + h * map_area,
                     floor_.begin() + (h + 1) * map_area, OCCUPANCY::FREE));
    }

    const auto floor_area_filter = static_cast<int>(*std::max_element(floor_runnable_area_.begin(), floor_runnable_area_.end()) * params_.floor_area_thresh_rate);
    auto is_peak = [H](const int h, const std::vector<int>& area){
      return (h == 0 || area[h - 1] <= area[h]) && (h == H - 1 || area[h + 1] <= area[h]);
    };

    for (int h = 0; h < H; h++)
    {
      if (hist_[h] > min_points && is_peak(h, floor_runnable_area_) && floor_runnable_area_[h] > floor_area_filter)
      {
        nav_msgs::msg::OccupancyGrid map;
        map.info = mmd;
        map.info.origin.position.z = (h + range.z_min) * params_.grid;
        map.header = msg->header;
        map.data.resize(map_area);
        std::copy_n(floor_.begin() + h * map_area, map_area, map.data.begin());
        maps.push_back(std::move(map));
      }
    }
    if (maps.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No valid floor candidates found");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Floor candidates: %ld", maps.size());
    const double merge_threshold_k = 1.5;
    const auto merge_threshold = params_.grid * merge_threshold_k;
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

        if (p == OCCUPANCY::FREE)
        {
          prev.data[j] = OCCUPANCY::UNKNOWN;
          if (c != OCCUPANCY::FREE)
          {
            cur.data[j] = OCCUPANCY::FREE;
          }
        }
      }
    }

    std::vector<int> map_runnable_area(maps.size());
    for (size_t i = 0; i < maps.size(); i++)
    {
      map_runnable_area[i] =
        static_cast<int>(std::count(maps[i].data.begin(), maps[i].data.end(), OCCUPANCY::FREE));
    }

    if (rcutils_logging_get_logger_effective_level(this->get_logger().get_name()) <= RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG)
    {
      for (auto h = H - 1; h >= 0; h--)
      {
        std::string bar;
        const auto bar_len = (hist_[h] * 16) / hist_max;
        bar.reserve(16);
        for (auto j = 0; j <= 16; j++)
        {
          bar.push_back(j <= bar_len ? '#' : ' ');
        }
        const auto z = (h + range.z_min) * params_.grid;
        if (floor_runnable_area_[h] == 0)
        {
          RCLCPP_DEBUG(this->get_logger(), "%6.2f %s  (%7d points)", z, bar.c_str(), hist_[h]);
        }
        else
        {
          RCLCPP_DEBUG(this->get_logger(), "%6.2f %s  (%7d points, %5.2f m^2 of floor))", z, bar.c_str(), hist_[h], floor_runnable_area_[h] * cell_area);
        }
      }
    }

    int floor_num = 0;
    map_organizer_msgs::msg::OccupancyGridArray map_array;
    pub_maps_.clear();
    for (size_t k = 0 ; k < maps.size(); k++)
    {
      auto& map = maps[k];
      if (map_runnable_area[k] * cell_area < params_.min_floor_area)
      {
        RCLCPP_WARN(this->get_logger(), "floor %d (%5.2fm^2), h = %0.2fm skipped",
                 floor_num, map_runnable_area[k] * cell_area, map.info.origin.position.z);
        continue;
      }

      {
        const auto& src = map.data;

        const int& mW = map.info.width;
        const int& mH = map.info.height;
        const int R = 6;
        std::vector<int> dist(mW * mH, std::numeric_limits<int>::max());

        for (auto i = 0; i < mW * mH; i++) {
          if (src[i] == OCCUPANCY::OCCUPIED)
            dist[i] = OCCUPANCY::FREE;
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
            map.data[i] = OCCUPANCY::FREE;
          }
        }
      }
      for (size_t j = 0; j < maps.size(); j++)
      {
        map_runnable_area[j] =
          static_cast<int>(std::count(maps[j].data.begin(), maps[j].data.end(), OCCUPANCY::FREE));
      }
      const std::string name = "map" + std::to_string(floor_num);
      pub_maps_[name] = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/" + name, rclcpp::QoS(1).transient_local());
      pub_maps_[name]->publish(map);
      map_array.maps.push_back(map);
      RCLCPP_DEBUG(this->get_logger(), "floor %d (%5.2fm^2), h = %0.2fm",
               floor_num, map_runnable_area[k] * cell_area, map.info.origin.position.z);
      floor_num++;
    }
    pub_map_array_->publish(map_array);
  }
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_organizer::PointcloudToMapsNode)
