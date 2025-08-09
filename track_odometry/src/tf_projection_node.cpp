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

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include <track_odometry/tf_projection.h>

class TfProjectionNode : public rclcpp::Node
{
private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double rate_;
  double tf_tolerance_;
  bool flat_;
  bool project_posture_;
  bool align_all_posture_to_source_;

  std::string source_frame_;
  std::string projection_surface_frame_;
  std::string parent_frame_;
  std::string projected_frame_;

public:
  TfProjectionNode()
    : Node("tf_projection")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (this->has_parameter("base_link_frame") ||
        this->has_parameter("projection_frame") ||
        this->has_parameter("target_frame") ||
        this->has_parameter("frame"))
    {
      RCLCPP_ERROR(this->get_logger(),
          "tf_projection parameters \"base_link_frame\", \"projection_frame\", \"target_frame\", and \"frame\" "
          "are replaced by \"source_frame\", \"projection_surface_frame\", \"parent_frame\", and \"projected_frame\"");

      source_frame_ = this->declare_parameter("base_link_frame", std::string("base_link"));
      projection_surface_frame_ = this->declare_parameter("projection_frame", std::string("map"));
      parent_frame_ = this->declare_parameter("target_frame", std::string("map"));
      projected_frame_ = this->declare_parameter("frame", std::string("base_link_projected"));
    }
    else
    {
      source_frame_ = this->declare_parameter("source_frame", std::string("base_link"));
      projection_surface_frame_ = this->declare_parameter("projection_surface_frame", std::string("map"));
      parent_frame_ = this->declare_parameter("parent_frame", std::string("map"));
      projected_frame_ = this->declare_parameter("projected_frame", std::string("base_link_projected"));
    }

    rate_ = this->declare_parameter("hz", 10.0);
    tf_tolerance_ = this->declare_parameter("tf_tolerance", 0.1);
    flat_ = this->declare_parameter("flat", false);

    project_posture_ = this->declare_parameter("project_posture", false);
    align_all_posture_to_source_ = this->declare_parameter("align_all_posture_to_source", false);
  }
  void process()
  {
    tf2::Stamped<tf2::Transform> trans;
    tf2::Stamped<tf2::Transform> trans_target;
    try
    {
      tf2::fromMsg(
          tf_buffer_->lookupTransform(projection_surface_frame_, source_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
          trans);
      tf2::fromMsg(
          tf_buffer_->lookupTransform(parent_frame_, projection_surface_frame_, trans.stamp_, tf2::durationFromSec(0.1)),
          trans_target);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", e.what());
      return;
    }

    if (trans.stamp_ != tf2::TimePointZero)
      trans.stamp_ += tf2::durationFromSec(tf_tolerance_);

    if (project_posture_)
    {
      if (align_all_posture_to_source_)
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
        parent_frame_);

    geometry_msgs::msg::TransformStamped trans_out = tf2::toMsg(result);
    if (flat_)
    {
      const double yaw = tf2::getYaw(trans_out.transform.rotation);
      trans_out.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    }
    trans_out.child_frame_id = projected_frame_;

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
  void spin()
  {
    rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_), std::bind(&TfProjectionNode::cbTimer, this));
    rclcpp::spin(shared_from_this());
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto proj = std::make_shared<TfProjectionNode>();
  proj->spin();

  return 0;
}
