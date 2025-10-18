/*
 * Copyright (c) 2014-2018, the neonavigation authors
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
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <string>
#include <utility>
#include <vector>

#include <costmap_cspace_msgs/msg/c_space3_d.hpp>
#include <costmap_cspace_msgs/msg/c_space3_d_update.hpp>

#include <costmap_cspace/costmap_3d.h>

#include <costmap_cspace/costmap_3d_parameters.hpp>

namespace costmap_cspace
{
class Costmap3DOFNode : public rclcpp::Node
{
protected:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> sub_map_overlay_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr pub_costmap_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr pub_costmap_update_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_footprint_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_footprint_;

  costmap_cspace::Costmap3d::SharedPtr costmap_;
  std::vector<
      std::pair<nav_msgs::msg::OccupancyGrid::ConstSharedPtr,
                costmap_cspace::Costmap3dLayerBase::SharedPtr>>
      map_buffer_;

  std::shared_ptr<costmap_3d::ParamListener> param_listener_;
  costmap_3d::Params params_;

  void cbMap(
      const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg,
      const costmap_cspace::Costmap3dLayerBase::SharedPtr map)
  {
    if (map->getAngularGrid() <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "params_.ang_resolution is not set.");
      std::runtime_error("params_.ang_resolution is not set.");
    }
    RCLCPP_INFO(this->get_logger(), "2D costmap received");

    map->setBaseMap(msg);
    RCLCPP_DEBUG(this->get_logger(), "C-Space costmap generated");

    if (map_buffer_.size() > 0)
    {
      for (auto& buf : map_buffer_)
        cbMapOverlay(buf.first, buf.second);
      RCLCPP_INFO(this->get_logger(), "%ld buffered costmaps processed", map_buffer_.size());
      map_buffer_.clear();
    }
  }
  void cbMapOverlay(
      const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg,
      const costmap_cspace::Costmap3dLayerBase::SharedPtr map)
  {
    RCLCPP_DEBUG(this->get_logger(), "Overlay 2D costmap received");

    auto map_msg = map->getMap();
    if (map_msg->info.width < 1 ||
        map_msg->info.height < 1)
    {
      map_buffer_.push_back(
          std::pair<nav_msgs::msg::OccupancyGrid::ConstSharedPtr,
                    costmap_cspace::Costmap3dLayerBase::SharedPtr>(msg, map));
      return;
    }

    map->processMapOverlay(msg, true);
    RCLCPP_DEBUG(this->get_logger(), "C-Space costmap updated");
  }
  bool cbUpdateStatic(
      const costmap_cspace::CSpace3DMsg::SharedPtr& map)
  {
    publishDebug(*map);
    pub_costmap_->publish<costmap_cspace_msgs::msg::CSpace3D>(*map);
    return true;
  }
  bool cbUpdate(
      const costmap_cspace::CSpace3DMsg::SharedPtr& map,
      const costmap_cspace_msgs::msg::CSpace3DUpdate::SharedPtr& update)
  {
    if (update)
    {
      publishDebug(*map);
      pub_costmap_update_->publish(*update);
      if (update->width * update->height * update->angle == 0)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
            5000, "Updated region of the costmap is empty. "
               "The position may be out-of-boundary, or input map is wrong.");
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to update the costmap.");
    }
    return true;
  }
  void publishDebug(const costmap_cspace_msgs::msg::CSpace3D& map)
  {
    if (pub_debug_->get_subscription_count() == 0)
      return;
    sensor_msgs::msg::PointCloud pc;
    pc.header = map.header;
    pc.header.stamp = this->now();
    for (size_t yaw = 0; yaw < map.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < map.info.width * map.info.height; i++)
      {
        int gx = i % map.info.width;
        int gy = i / map.info.width;
        if (map.data[i + yaw * map.info.width * map.info.height] < 100)
          continue;
        geometry_msgs::msg::Point32 p;
        p.x = gx * map.info.linear_resolution + map.info.origin.position.x;
        p.y = gy * map.info.linear_resolution + map.info.origin.position.y;
        p.z = yaw * 0.1;
        pc.points.push_back(p);
      }
    }
    pub_debug_->publish(pc);
  }
  void cbPublishFootprint(const geometry_msgs::msg::PolygonStamped msg)
  {
    auto footprint = msg;
    footprint.header.stamp = this->now();
    pub_footprint_->publish(footprint);
  }

public:
  Costmap3DOFNode(const rclcpp::NodeOptions& options) : Node("costmap_3d", options)
  {
    pub_costmap_ = this->create_publisher<costmap_cspace_msgs::msg::CSpace3D>(
        "costmap",
        rclcpp::QoS(1).transient_local());
    pub_costmap_update_ = this->create_publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>(
        "costmap_update",
        rclcpp::QoS(1).transient_local());
    pub_footprint_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("~/footprint", rclcpp::QoS(2).transient_local());
    pub_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud>("~/debug", rclcpp::QoS(1).transient_local());

    param_listener_ = std::make_shared<costmap_3d::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    costmap_cspace::Polygon footprint;
    try
    {
      footprint = costmap_cspace::Polygon(params_.footprint);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(this->get_logger(), "Invalid footprint");
      throw e;
    }

    costmap_ = std::make_shared<costmap_cspace::Costmap3d>(params_.ang_resolution);
    Costmap3dLayerBase::LayerConfig root_layer_config;
    root_layer_config.footprint = params_.footprint;

    auto root_layer = costmap_->addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
    root_layer->setExpansion(params_.linear_expand, params_.linear_spread, params_.linear_spread_min_cost);
    root_layer->setFootprint(footprint);

    {
      for (const auto& [name, static_layer] : params_.static_layers_map)
      {
        costmap_cspace::Costmap3dLayerBase::LayerConfig layer_xml;
        layer_xml.name = name;
        RCLCPP_INFO(this->get_logger(), "New static layer: %s", name.c_str());

        layer_xml.footprint = static_layer.footprint.empty() ? params_.footprint : static_layer.footprint;

        auto layer = costmap_cspace::Costmap3dLayerClassLoader::loadClass(static_layer.type);
        costmap_->addLayer(layer, static_layer.overlay_mode);
        layer->loadConfig(layer_xml, *this);

        sub_map_overlay_.push_back(this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            layer_xml.name, 1,
            [this, layer](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){return this->cbMapOverlay(msg, layer);}));
      }
    }

    auto static_output_layer = costmap_->addLayer<costmap_cspace::Costmap3dStaticLayerOutput>();
    static_output_layer->setHandler([this](const costmap_cspace::CSpace3DMsg::SharedPtr& map){ return cbUpdateStatic(map);});

    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local(),
        [this, root_layer](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){return cbMap(msg, root_layer);});

    {
      for (const auto& [name, dynamic_layer] : params_.layers_map)
      {
        auto layer_xml = costmap_cspace::Costmap3dLayerBase::LayerConfig();
        layer_xml.name = name;
        RCLCPP_INFO(this->get_logger(), "New layer: %s", layer_xml.name.c_str());

        layer_xml.footprint = dynamic_layer.footprint.empty() ? params_.footprint : dynamic_layer.footprint;

        auto layer = costmap_cspace::Costmap3dLayerClassLoader::loadClass(dynamic_layer.type);
        costmap_->addLayer(layer, dynamic_layer.overlay_mode);
        layer->loadConfig(layer_xml, *this);

        sub_map_overlay_.push_back(this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            layer_xml.name, 1,
            [this, layer](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){return cbMapOverlay(msg, layer);}));
      }
    }

    auto update_output_layer = costmap_->addLayer<costmap_cspace::Costmap3dUpdateLayerOutput>();
    update_output_layer->setHandler([this](const costmap_cspace::CSpace3DMsg::SharedPtr& map,const costmap_cspace_msgs::msg::CSpace3DUpdate::SharedPtr& update){ return cbUpdate(map,update); });

    const geometry_msgs::msg::PolygonStamped footprint_msg = footprint.toMsg();
    timer_footprint_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0),
        [=](){cbPublishFootprint(footprint_msg);});
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_cspace::Costmap3DOFNode)