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

#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include <track_odometry/tf_projection.h>

#include <track_odometry/tf_projection_parameters.hpp>

namespace track_odometry
{

class TfProjectionNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<tf_projection_node::ParamListener> param_listener_;
  tf_projection_node::Params params_;

public:
  TfProjectionNode(const rclcpp::NodeOptions& options) : Node("tf_projection", options)
  {
    param_listener_ = std::make_shared<tf_projection_node::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.hz), std::bind(&TfProjectionNode::cbTimer, this));
  }
  void process()
  {
    tf2::Stamped<tf2::Transform> trans;
    tf2::Stamped<tf2::Transform> trans_target;
    try
    {
      tf2::fromMsg(
          tf_buffer_->lookupTransform(params_.projection_surface_frame, params_.source_frame, tf2::TimePointZero, tf2::durationFromSec(0.1)),
          trans);
      tf2::fromMsg(
          tf_buffer_->lookupTransform(params_.parent_frame, params_.projection_surface_frame, trans.stamp_, tf2::durationFromSec(0.1)),
          trans_target);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", e.what());
      return;
    }

    if (trans.stamp_ != tf2::TimePointZero)
      trans.stamp_ += tf2::durationFromSec(params_.tf_tolerance);

    if (params_.project_posture)
    {
      if (params_.align_all_posture_to_source)
      {
        const tf2::Quaternion rot(trans.getRotation());
        const tf2::Quaternion rot_yaw(tf2::Vector3(0.0, 0.0, 1.0), tf2::getYaw(rot));
        const tf2::Transform rot_inv(rot_yaw * rot.inverse());
        trans.setData(rot_inv * trans);
      }
      else
      {
        const float yaw = tf2::getYaw(trans.getRotation());
        trans.setRotation(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
      }
    }

    const tf2::Stamped<tf2::Transform> result(
        track_odometry::projectTranslation(trans, trans_target),
        trans.stamp_,
        params_.parent_frame);

    geometry_msgs::msg::TransformStamped trans_out = tf2::toMsg(result);
    if (params_.flat)
    {
      const double yaw = tf2::getYaw(trans_out.transform.rotation);
      trans_out.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    }
    trans_out.child_frame_id = params_.projected_frame;

    if (trans.stamp_ == tf2::TimePointZero)
    {
      tf_static_broadcaster_->sendTransform(trans_out);
    }
    else
    {
      tf_broadcaster_->sendTransform(trans_out);
    }
  }
  void cbTimer()
  {
    process();
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(track_odometry::TfProjectionNode)