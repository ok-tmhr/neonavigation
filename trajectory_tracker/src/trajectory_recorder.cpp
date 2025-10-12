/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
 */

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/empty.hpp>


class RecorderNode : public rclcpp::Node
{
public:
  RecorderNode();
  ~RecorderNode();
  void spin();

private:
  void clearPath(const std_srvs::srv::Empty::Request::SharedPtr req,
                 std_srvs::srv::Empty::Response::SharedPtr res);

  std::string frame_robot_;
  std::string frame_global_;
  double dist_interval_;
  double ang_interval_;
  bool store_time_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srs_clear_path_;

  nav_msgs::msg::Path path_;
};

RecorderNode::RecorderNode() : Node("trajectory_recorder")
{
  frame_robot_ = this->declare_parameter("frame_robot", std::string("base_link"));
  frame_global_ = this->declare_parameter("frame_global", std::string("map"));
  dist_interval_ = this->declare_parameter("dist_interval", 0.3);
  ang_interval_ = this->declare_parameter("ang_interval", 1.0);
  store_time_ = this->declare_parameter("store_time", false);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "path",
      rclcpp::QoS(10).transient_local());
  srs_clear_path_ = this->create_service<std_srvs::srv::Empty>("~/clear_path", std::bind(&RecorderNode::clearPath, this, std::placeholders::_1, std::placeholders::_2));

  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / 50.0),
    std::bind(&RecorderNode::spin, this));
}

RecorderNode::~RecorderNode()
{
}

float dist2d(geometry_msgs::msg::Point& a, geometry_msgs::msg::Point& b)
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

void RecorderNode::clearPath(const std_srvs::srv::Empty::Request::SharedPtr /* req */,
                             std_srvs::srv::Empty::Response::SharedPtr /* res */)
{
  path_.poses.clear();
}

void RecorderNode::spin()
{
  path_.header.frame_id = frame_global_;

  {
    rclcpp::Time now = rclcpp::Time(0L, RCL_ROS_TIME);
    if (store_time_)
      now = this->now();
    tf2::Stamped<tf2::Transform> transform;
    try
    {
      tf2::fromMsg(
          tfbuf_->lookupTransform(frame_global_, frame_robot_, now, rclcpp::Duration::from_seconds(0.2)), transform);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(), "TF exception: %s", e.what());
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    tf2::Quaternion q;
    transform.getBasis().getRotation(q);
    pose.pose.orientation = tf2::toMsg(q);
    tf2::Vector3 origin = transform.getOrigin();
    pose.pose.position.x = origin.x();
    pose.pose.position.y = origin.y();
    pose.pose.position.z = origin.z();
    pose.header.frame_id = frame_global_;
    pose.header.stamp = now;

    path_.header.stamp = now;

    if (path_.poses.size() == 0)
    {
      path_.poses.push_back(pose);
      pub_path_->publish(path_);
    }
    else if (dist2d(path_.poses.back().pose.position, pose.pose.position) > dist_interval_)
    {
      path_.poses.push_back(pose);
      pub_path_->publish(path_);
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto rec = std::make_shared<RecorderNode>();
  rclcpp::spin(rec);

  return 0;
}
