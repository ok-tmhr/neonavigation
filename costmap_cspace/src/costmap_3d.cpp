/*
 * Copyright (c) 2014-2018, the neonavigation authors
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
#include <neonavigation_common/compatibility.h>

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

  costmap_cspace::Costmap3d::Ptr costmap_;
  std::vector<
      std::pair<nav_msgs::msg::OccupancyGrid::ConstPtr,
                costmap_cspace::Costmap3dLayerBase::Ptr>>
      map_buffer_;

  void cbMap(
      const nav_msgs::msg::OccupancyGrid::ConstPtr& msg,
      const costmap_cspace::Costmap3dLayerBase::Ptr map)
  {
    if (map->getAngularGrid() <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "ang_resolution is not set.");
      std::runtime_error("ang_resolution is not set.");
    }
    RCLCPP_INFO(this->get_logger(), "2D costmap received");

    map->setBaseMap(msg);
    RCLCPP_DEBUG(this->get_logger(), "C-Space costmap generated");

    if (map_buffer_.size() > 0)
    {
      for (auto& map : map_buffer_)
        cbMapOverlay(map.first, map.second);
      RCLCPP_INFO(this->get_logger(), "%ld buffered costmaps processed", map_buffer_.size());
      map_buffer_.clear();
    }
  }
  void cbMapOverlay(
      const nav_msgs::msg::OccupancyGrid::ConstPtr& msg,
      const costmap_cspace::Costmap3dLayerBase::Ptr map)
  {
    RCLCPP_DEBUG(this->get_logger(), "Overlay 2D costmap received");

    auto map_msg = map->getMap();
    if (map_msg->info.width < 1 ||
        map_msg->info.height < 1)
    {
      map_buffer_.push_back(
          std::pair<nav_msgs::msg::OccupancyGrid::ConstPtr,
                    costmap_cspace::Costmap3dLayerBase::Ptr>(msg, map));
      return;
    }

    map->processMapOverlay(msg, true);
    RCLCPP_DEBUG(this->get_logger(), "C-Space costmap updated");
  }
  bool cbUpdateStatic(
      const costmap_cspace::CSpace3DMsg::Ptr& map)
  {
    publishDebug(*map);
    pub_costmap_->publish<costmap_cspace_msgs::msg::CSpace3D>(*map);
    return true;
  }
  bool cbUpdate(
      const costmap_cspace::CSpace3DMsg::Ptr& map,
      const costmap_cspace_msgs::msg::CSpace3DUpdate::Ptr& update)
  {
    if (update)
    {
      publishDebug(*map);
      pub_costmap_update_->publish(*update);
      if (update->width * update->height * update->angle == 0)
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(),
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
    pc.header.stamp = this->get_clock()->now();
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
    footprint.header.stamp = this->get_clock()->now();
    pub_footprint_->publish(footprint);
  }

  static costmap_cspace::MapOverlayMode getMapOverlayModeFromString(
      const std::string overlay_mode_str)
  {
    if (overlay_mode_str == "overwrite")
    {
      return costmap_cspace::MapOverlayMode::OVERWRITE;
    }
    else if (overlay_mode_str == "max")
    {
      return costmap_cspace::MapOverlayMode::MAX;
    }
    RCLCPP_FATAL(rclcpp::get_logger("costmap_3d"), "Unknown overlay_mode \"%s\"", overlay_mode_str.c_str());
    throw std::runtime_error("Unknown overlay_mode.");
  };

public:
  Costmap3DOFNode()
    : Node("costmap_3d")
  {
    pub_costmap_ = this->create_publisher<costmap_cspace_msgs::msg::CSpace3D>(
        "~/costmap", rclcpp::QoS(1).transient_local());
    pub_costmap_update_ = this->create_publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>(
        "~/costmap_update", rclcpp::QoS(1).transient_local());
    pub_footprint_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("~/footprint", rclcpp::QoS(2).transient_local());
    pub_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud>("~/debug", rclcpp::QoS(1).transient_local());

    int ang_resolution;
    ang_resolution = this->declare_parameter("ang_resolution", 16);

    auto footprint_str = this->declare_parameter("footprint", "");
    if (footprint_str.empty())
    {
      RCLCPP_FATAL(this->get_logger(), "Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified.");
    }
    costmap_cspace::Polygon footprint;
    try
    {
      footprint = costmap_cspace::Polygon(footprint_str);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(this->get_logger(), "Invalid footprint");
      throw e;
    }

    costmap_.reset(new costmap_cspace::Costmap3d(ang_resolution));

    auto root_layer = costmap_->addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
    float linear_expand;
    float linear_spread;
    linear_expand = this->declare_parameter("linear_expand", 0.2f);
    linear_spread = this->declare_parameter("linear_spread", 0.5f);
    int linear_spread_min_cost;
    linear_spread_min_cost = this->declare_parameter("linear_spread_min_cost", 0);
    root_layer->setExpansion(linear_expand, linear_spread, linear_spread_min_cost);
    root_layer->setFootprint(footprint);

    RCLCPP_DEBUG(this->get_logger(), "static_layers");
    auto static_layers = this->declare_parameter("static_layers", std::vector<std::string>{});
    for (int i = 0; i < static_layers.size(); ++i)
      {
      costmap_cspace::Costmap3dLayerBase::LayerConfig layer_config;
      layer_config.name = this->declare_parameter(static_layers[i] + ".name", "");
      RCLCPP_INFO(this->get_logger(), "New static layer: %s", layer_config.name.c_str());

        costmap_cspace::MapOverlayMode overlay_mode(costmap_cspace::MapOverlayMode::MAX);
      layer_config.overlay_mode = this->declare_parameter(layer_config.name + ".overlay_mode", "");
      if (!layer_config.overlay_mode.empty())
          overlay_mode = getMapOverlayModeFromString(
            layer_config.overlay_mode);
        else
        RCLCPP_WARN(this->get_logger(), "overlay_mode of the static layer is not specified. Using MAX mode.");

      layer_config.type = this->declare_parameter(layer_config.name + ".type", "");
      if (layer_config.type.empty())
        {
        RCLCPP_FATAL(this->get_logger(), "Layer type is not specified.");
          throw std::runtime_error("Layer type is not specified.");
        }

      layer_config.footprint = this->declare_parameter(layer_config.name + ".footprint", footprint_str);

        costmap_cspace::Costmap3dLayerBase::Ptr layer =
          costmap_cspace::Costmap3dLayerClassLoader::loadClass(layer_config.type);
        costmap_->addLayer(layer, overlay_mode);
      layer->loadConfig(layer_config);

      sub_map_overlay_.push_back(this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          layer_config.name, 1,
          [=](const nav_msgs::msg::OccupancyGrid::ConstPtr& msg){return Costmap3DOFNode::cbMapOverlay(msg, layer);}));
    }

    auto static_output_layer = costmap_->addLayer<costmap_cspace::Costmap3dStaticLayerOutput>();
    static_output_layer->setHandler([&](const costmap_cspace::CSpace3DMsg::Ptr &map){return Costmap3DOFNode::cbUpdateStatic(map);});

    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 1,
        [=](const nav_msgs::msg::OccupancyGrid::ConstPtr& msg){return Costmap3DOFNode::cbMap(msg, root_layer);});

    auto layers = this->declare_parameter("layers", std::vector<std::string>{});
    if (layers.size() > 0)
      {
      costmap_cspace::Costmap3dLayerBase::LayerConfig layer_config;
      
      for (int i = 0; i < layers.size(); ++i)
      {
        layer_config.name = this->declare_parameter(layers[i] + ".name", "");
        RCLCPP_INFO(this->get_logger(), "New layer: %s", layer_config.name.c_str());

        costmap_cspace::MapOverlayMode overlay_mode(costmap_cspace::MapOverlayMode::MAX);
        layer_config.overlay_mode = this->declare_parameter(layer_config.name + ".overlay_mode", "");
        if (!layer_config.overlay_mode.empty())
          overlay_mode = getMapOverlayModeFromString(
              layer_config.overlay_mode);
        else
          RCLCPP_WARN(this->get_logger(), "overlay_mode of the layer is not specified. Using MAX mode.");

        layer_config.type = this->declare_parameter(layer_config.name + ".type", "");
        if (layer_config.type.empty())
        {
          RCLCPP_FATAL(this->get_logger(), "Layer type is not specified.");
          throw std::runtime_error("Layer type is not specified.");
        }

        layer_config.footprint = this->declare_parameter(layer_config.name + ".footprint", footprint_str);

        costmap_cspace::Costmap3dLayerBase::Ptr layer =
            costmap_cspace::Costmap3dLayerClassLoader::loadClass(layer_config.type);
        costmap_->addLayer(layer, overlay_mode);
        layer->loadConfig(layer_config);

        sub_map_overlay_.push_back(this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            layer_config.name, 1,
            [=](const nav_msgs::msg::OccupancyGrid::ConstPtr& msg){return Costmap3DOFNode::cbMapOverlay(msg, layer);}));
      }
    }
    else
    {
      // Single layer mode for backward-compatibility
      costmap_cspace::MapOverlayMode overlay_mode;
      std::string overlay_mode_str;
      overlay_mode_str = this->declare_parameter("overlay_mode", std::string("max"));
      if (overlay_mode_str.compare("overwrite") == 0)
        overlay_mode = costmap_cspace::MapOverlayMode::OVERWRITE;
      else if (overlay_mode_str.compare("max") == 0)
        overlay_mode = costmap_cspace::MapOverlayMode::MAX;
      else
      {
        RCLCPP_FATAL(this->get_logger(), "Unknown overlay_mode \"%s\"", overlay_mode_str.c_str());
        throw std::runtime_error("Unknown overlay_mode.");
      }
      RCLCPP_INFO(this->get_logger(), "costmap_3d: %s mode", overlay_mode_str.c_str());

      costmap_cspace::Costmap3dLayerBase::LayerConfig layer_config;
      layer_config.footprint = footprint_str;
      layer_config.linear_expand = linear_expand;
      layer_config.linear_spread = linear_spread;

      auto layer = costmap_->addLayer<costmap_cspace::Costmap3dLayerFootprint>(overlay_mode);
      layer->loadConfig(layer_config);
      sub_map_overlay_.push_back(this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "map_overlay", 1,
          [=](const nav_msgs::msg::OccupancyGrid::ConstPtr& msg){return Costmap3DOFNode::cbMapOverlay(msg, layer);}));
    }

    auto update_output_layer = costmap_->addLayer<costmap_cspace::Costmap3dUpdateLayerOutput>();
    update_output_layer->setHandler([&](
      const costmap_cspace::CSpace3DMsg::Ptr& map,
      const costmap_cspace_msgs::msg::CSpace3DUpdate::Ptr& update)
      {
        return Costmap3DOFNode::cbUpdate(map, update);
      });

    const geometry_msgs::msg::PolygonStamped footprint_msg = footprint.toMsg();
    timer_footprint_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0),
        [=](){return Costmap3DOFNode::cbPublishFootprint(footprint_msg);});
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto cm = std::make_shared<Costmap3DOFNode>();
  rclcpp::spin(cm);

  return 0;
}
