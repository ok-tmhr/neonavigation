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
#include <laser_geometry/laser_geometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <limits>
#include <string>

#include <costmap_cspace/pointcloud_accumulator.h>

class LaserscanToMapNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

  nav_msgs::msg::OccupancyGrid map;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  laser_geometry::LaserProjection projector_;
  rclcpp::Time published_;
  rclcpp::Duration publish_interval_;

  double z_min_, z_max_;
  std::string global_frame_;
  std::string robot_frame_;

  unsigned int width_;
  unsigned int height_;
  float origin_x_;
  float origin_y_;

  costmap_cspace::PointcloudAccumulator<sensor_msgs::msg::PointCloud2> accum_;

public:
  LaserscanToMapNode()
    : Node("laserscan_to_map")
    , tfbuf_(this->get_clock())
    , tfl_(tfbuf_)
    , publish_interval_(rclcpp::Duration::from_seconds(0.))
  {
    // neonavigation_common::compat::checkCompatMode();
    z_min_ = this->declare_parameter("z_min", std::numeric_limits<double>::lowest());
    z_max_ = this->declare_parameter("z_max", std::numeric_limits<double>::max());
    global_frame_ = this->declare_parameter("global_frame", std::string("map"));
    robot_frame_ = this->declare_parameter("robot_frame", std::string("base_link"));

    double accum_duration;
    accum_duration = this->declare_parameter("accum_duration", 1.0);
    accum_.reset(std::chrono::duration<double>(accum_duration));

    pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "~/map", rclcpp::QoS(1).transient_local());
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 2, std::bind(&LaserscanToMapNode::cbScan, this, std::placeholders::_1));

    int width_param;
    width_param = this->declare_parameter("width", 30);
    height_ = width_ = width_param;
    map.header.frame_id = global_frame_;

    double resolution;
    resolution = this->declare_parameter("resolution", 0.1);
    map.info.resolution = resolution;
    map.info.width = width_;
    map.info.height = height_;
    map.data.resize(map.info.width * map.info.height);

    double hz;
    hz = this->declare_parameter("hz", 1.0);
    publish_interval_ = std::chrono::duration<double>(1.0 / hz);
  }

private:
  void cbScan(const sensor_msgs::msg::LaserScan::ConstPtr& scan)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::msg::PointCloud2 cloud_global;
    projector_.projectLaser(*scan, cloud);
    try
    {
      geometry_msgs::msg::TransformStamped trans = tfbuf_.lookupTransform(
          global_frame_, cloud.header.frame_id, cloud.header.stamp, std::chrono::duration<double>(0.5));
      tf2::doTransform(cloud, cloud_global, trans);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
    }
    accum_.push(costmap_cspace::PointcloudAccumurator<sensor_msgs::msg::PointCloud2>::Points(
        cloud_global, cloud_global.header.stamp));

    rclcpp::Time now = scan->header.stamp;
    if (published_ + publish_interval_ > now)
      return;
    published_ = now;

    float robot_z;
    try
    {
      tf2::Stamped<tf2::Transform> trans;
      tf2::fromMsg(tfbuf_.lookupTransform(global_frame_, robot_frame_, rclcpp::Time(0)), trans);

      auto pos = trans.getOrigin();
      float x = static_cast<int>(pos.x() / map.info.resolution) * map.info.resolution;
      float y = static_cast<int>(pos.y() / map.info.resolution) * map.info.resolution;
      map.info.origin.position.x = x - map.info.width * map.info.resolution * 0.5;
      map.info.origin.position.y = y - map.info.height * map.info.resolution * 0.5;
      map.info.origin.position.z = 0.0;
      map.info.origin.orientation.w = 1.0;
      origin_x_ = x - width_ * map.info.resolution * 0.5;
      origin_y_ = y - height_ * map.info.resolution * 0.5;
      robot_z = pos.z();
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      return;
    }
    for (auto& cell : map.data)
      cell = 0;

    for (auto& pc : accum_)
    {
      auto itr_x = sensor_msgs::PointCloud2ConstIterator<float>(pc, "x");
      auto itr_y = sensor_msgs::PointCloud2ConstIterator<float>(pc, "y");
      auto itr_z = sensor_msgs::PointCloud2ConstIterator<float>(pc, "z");
      for (; itr_x != itr_x.end(); ++itr_x, ++itr_y)
      {
        if (*itr_z - robot_z < z_min_ || z_max_ < *itr_z - robot_z)
          continue;
        unsigned int x = int(
            (*itr_x - map.info.origin.position.x) / map.info.resolution);
        unsigned int y = int(
            (*itr_y - map.info.origin.position.y) / map.info.resolution);
        if (x >= map.info.width || y >= map.info.height)
          continue;
        map.data[x + y * map.info.width] = 100;
      }
    }

    pub_map_->publish(map);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto conv = std::make_shared<LaserscanToMapNode>();
  rclcpp::spin(conv);

  return 0;
}
