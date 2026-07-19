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

#pragma once

#include <memory>

#include <costmap_cspace_msgs/msg/c_space3_d.hpp>
#include <costmap_cspace_msgs/msg/c_space3_d_update.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <costmap_cspace/costmap_3d_layer/base.h>
#include <costmap_cspace/costmap_3d_layer/footprint.h>

#include <costmap_cspace/plain_parameters.hpp>
namespace costmap_cspace
{
class Costmap3dLayerPlain : public Costmap3dLayerFootprint
{
public:
  using SharedPtr = std::shared_ptr<Costmap3dLayerPlain>;

  Costmap3dLayerPlain()
  {
    Polygon footprint;
    footprint.v.resize(3);
    for (auto& p : footprint.v)
    {
      p[0] = p[1] = 0.0;
    }
    setFootprint(footprint);
  }
  void loadConfig(LayerConfig& config, rclcpp::Node& node)
  {
    auto param_listener = std::make_shared<plain::ParamListener>(node.get_node_parameters_interface(), config.name);
    auto params = param_listener->get_params();
    setExpansion(
      static_cast<float>(params.linear_expand),
      static_cast<float>(params.linear_spread),
      static_cast<int>(params.linear_spread_min_cost)
    );
  }
};
}  // namespace costmap_cspace

