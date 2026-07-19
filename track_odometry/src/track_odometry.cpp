/*
 * Copyright (c) 2014-2019, the neonavigation authors
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
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <track_odometry/kalman_filter1.h>

#include <track_odometry/track_odometry_parameters.hpp>

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}
Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion& a)
{
  return Eigen::Quaterniond(a.w, a.x, a.y, a.z);
}
geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& a)
{
  geometry_msgs::msg::Point b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}
geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d& a)
{
  geometry_msgs::msg::Vector3 b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}

namespace track_odometry
{

class TrackOdometryNode : public rclcpp::Node
{
private:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> sub_imu_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_reset_z_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<track_odometry::ParamListener> param_listener_;
  nav_msgs::msg::Odometry odom_prev_;
  nav_msgs::msg::Odometry odomraw_prev_;

  track_odometry::Params params_;

  std::string base_link_id_;
  std::string base_link_id_overwrite_;

  sensor_msgs::msg::Imu imu_;
  double gyro_zero_[3];

  double dist_;

  track_odometry::KalmanFilter1 slip_;

  bool has_imu_;
  bool has_odom_;

  void cbResetZ(const std_msgs::msg::Float32::SharedPtr msg)
  {
    odom_prev_.pose.pose.position.z = msg->data;
  }
  void cbOdomImu(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
  {
    RCLCPP_DEBUG(this->get_logger(),
        "Synchronized timestamp: odom %0.3f, imu %0.3f",
        tf2_ros::timeToSec(odom_msg->header.stamp),
        tf2_ros::timeToSec(imu_msg->header.stamp));
    cbImu(imu_msg);
    cbOdom(odom_msg);
  }
  void cbImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (base_link_id_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "base_link id is not specified.");
      return;
    }

    imu_.header = msg->header;
    try
    {
      auto trans = tf_buffer_->lookupTransform(
          base_link_id_, msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));

      geometry_msgs::msg::Vector3Stamped vin, vout;
      vin.header = imu_.header;
      vin.header.stamp = rclcpp::Time();
      vin.vector = msg->linear_acceleration;
      tf2::doTransform(vin, vout, trans);
      imu_.linear_acceleration = vout.vector;

      vin.header = imu_.header;
      vin.header.stamp = rclcpp::Time();
      vin.vector = msg->angular_velocity;
      tf2::doTransform(vin, vout, trans);
      imu_.angular_velocity = vout.vector;

      tf2::Stamped<tf2::Quaternion> qin, qout;
      geometry_msgs::msg::QuaternionStamped qmin, qmout;
      qmin.header = imu_.header;
      qmin.quaternion = msg->orientation;
      tf2::fromMsg(qmin, qin);

      auto axis = qin.getAxis();
      auto angle = qin.getAngle();
      geometry_msgs::msg::Vector3Stamped axis2;
      geometry_msgs::msg::Vector3Stamped axis1;
      axis1.vector = tf2::toMsg(axis);
      axis1.header.stamp = rclcpp::Time();
      axis1.header.frame_id = qin.frame_id_;
      tf2::doTransform(axis1, axis2, trans);

      tf2::fromMsg(axis2.vector, axis);
      qout.setData(tf2::Quaternion(axis, angle));
      qout.stamp_ = qin.stamp_;
      qout.frame_id_ = base_link_id_;

      qmout = tf2::toMsg(qout);
      imu_.orientation = qmout.quaternion;

      has_imu_ = true;
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      has_imu_ = false;
      return;
    }
  }
  void cbOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    nav_msgs::msg::Odometry odom = *msg;
    if (has_odom_)
    {
      const double dt = tf2_ros::timeToSec(odom.header.stamp) - tf2_ros::timeToSec(odomraw_prev_.header.stamp);
      if (base_link_id_overwrite_.empty())
      {
        base_link_id_ = odom.child_frame_id;
      }

      if (!has_imu_)
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "IMU data not received");
        return;
      }

      double slip_ratio = 1.0;
      odom.header.stamp = rclcpp::Duration::from_seconds(params_.tf_tolerance) + odom.header.stamp;
      odom.twist.twist.angular = imu_.angular_velocity;
      odom.pose.pose.orientation = imu_.orientation;

      double w_imu = imu_.angular_velocity.z;
      const double w_odom = msg->twist.twist.angular.z;

      if (w_imu * w_odom < 0 && !params_.enable_negative_slip)
        w_imu = w_odom;

      slip_.predict(-slip_.x_ * dt * params_.predict_filter_tc, dt * params_.sigma_predict);
      if (std::abs(w_odom) > params_.sigma_odom * 3)
      {
        // non-kf mode: calculate slip_ratio if angular vel < 3*sigma
        slip_ratio = w_imu / w_odom;
      }

      const double slip_ratio_per_angvel =
          (w_odom - w_imu) / (w_odom * std::abs(w_odom));
      double slip_ratio_per_angvel_sigma =
          params_.sigma_odom * std::abs(2.0 * w_odom * params_.sigma_odom / std::pow(w_odom * w_odom - params_.sigma_odom * params_.sigma_odom, 2));
      if (std::abs(w_odom) < params_.sigma_odom)
        slip_ratio_per_angvel_sigma = std::numeric_limits<double>::infinity();

      slip_.measure(slip_ratio_per_angvel, slip_ratio_per_angvel_sigma);

      if (params_.debug)
      {
        RCLCPP_DEBUG(this->get_logger(), "%0.3f %0.3f  %0.3f  %0.3f %0.3f  %0.3f  %0.3f\n",
               imu_.angular_velocity.z,
               msg->twist.twist.angular.z,
               slip_ratio,
               slip_.x_, slip_.sigma_,
               odom.twist.twist.linear.x, dist_);
      }
      dist_ += odom.twist.twist.linear.x * dt;

      const Eigen::Vector3d diff = toEigen(msg->pose.pose.position) - toEigen(odomraw_prev_.pose.pose.position);
      Eigen::Vector3d v =
          toEigen(odom.pose.pose.orientation) * toEigen(msg->pose.pose.orientation).inverse() * diff;
      if (params_.use_kf)
        v *= 1.0 - slip_.x_;
      else
        v *= slip_ratio;

      odom.pose.pose.position = toPoint(toEigen(odom_prev_.pose.pose.position) + v);
      if (params_.z_filter_timeconst > 0)
        odom.pose.pose.position.z *= 1.0 - (dt / params_.z_filter_timeconst);

      odom.child_frame_id = base_link_id_;
      pub_odom_->publish(odom);

      geometry_msgs::msg::TransformStamped odom_trans;

      odom_trans.header = odom.header;
      odom_trans.child_frame_id = base_link_id_;
      odom_trans.transform.translation = toVector3(toEigen(odom.pose.pose.position));
      odom_trans.transform.rotation = odom.pose.pose.orientation;
      if (params_.publish_tf)
        tf_broadcaster_->sendTransform(odom_trans);
    }
    odomraw_prev_ = *msg;
    odom_prev_ = odom;
    has_odom_ = true;
  }

public:
  TrackOdometryNode(const rclcpp::NodeOptions& options) : Node("track_odometry", options)
  {
    param_listener_ = std::make_shared<track_odometry::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    const rclcpp::QoS transport_hints =
        params_.enable_tcp_no_delay ? rclcpp::QoS(50) : rclcpp::QoS(50).best_effort();

    if (params_.without_odom)
    {
      sub_imu_raw_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "imu/data",
          64, [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg){ cbImu(msg); });
      base_link_id_ = this->declare_parameter("base_link_id", "base_link");
    }
    else
    {
      sub_odom_.subscribe(this, "odom_raw", transport_hints.get_rmw_qos_profile());
      sub_imu_.subscribe(this, "imu/data", transport_hints.get_rmw_qos_profile());

      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
              SyncPolicy(params_.sync_window), sub_odom_, sub_imu_);
      using std::placeholders::_1;
      using std::placeholders::_2;
      sync_->registerCallback(std::bind(&TrackOdometryNode::cbOdomImu, this, _1, _2));

      base_link_id_overwrite_ = this->declare_parameter("base_link_id", "");
    }

    sub_reset_z_ = this->create_subscription<std_msgs::msg::Float32>(
        "reset_odometry_z",
        1, [this](const std_msgs::msg::Float32::SharedPtr msg){ cbResetZ(msg); });
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 8);

    if (!base_link_id_overwrite_.empty())
    {
      base_link_id_ = base_link_id_overwrite_;
    }

    has_imu_ = false;
    has_odom_ = false;

    dist_ = 0;
    slip_.set(0.0, 0.1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (params_.without_odom)
    {
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(20), [this](){ cbTimer(); });
    }
  }
  void cbTimer()
  {
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = this->now();
    odom->header.frame_id = params_.odom_id;
    odom->child_frame_id = base_link_id_;
    odom->pose.pose.orientation.w = 1.0;
    cbOdom(odom);
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(track_odometry::TrackOdometryNode)