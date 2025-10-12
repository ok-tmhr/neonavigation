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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <safety_limiter_msgs/msg/safety_limiter_status.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <safety_limiter/safety_limiter_parameters.hpp>
#include <safety_limiter/utility.hpp>

namespace safety_limiter
{
pcl::PointXYZ operator-(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
  auto c = a;
  c.x -= b.x;
  c.y -= b.y;
  c.z -= b.z;
  return c;
}
pcl::PointXYZ operator+(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
  auto c = a;
  c.x += b.x;
  c.y += b.y;
  c.z += b.z;
  return c;
}
pcl::PointXYZ operator*(const pcl::PointXYZ& a, const float& b)
{
  auto c = a;
  c.x *= b;
  c.y *= b;
  c.z *= b;
  return c;
}
class SafetyLimiterNode : public rclcpp::Node
{
protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_cloud_;
  rclcpp::Publisher<safety_limiter_msgs::msg::SafetyLimiterStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_clouds_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_disable_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_watchdog_;
  rclcpp::TimerBase::SharedPtr predict_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  geometry_msgs::msg::Twist twist_;
  rclcpp::Time last_cloud_stamp_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_accum_;
  bool cloud_clear_;
  double vel_[2];
  double acc_[2];
  double tmax_;
  double r_lim_;
  double max_values_[2];
  double z_range_[2];
  float footprint_radius_;

  rclcpp::Time last_disable_cmd_;
  rclcpp::Duration hold_;
  rclcpp::Time hold_off_;
  rclcpp::Duration watchdog_interval_;

  bool watchdog_stop_;
  bool has_cloud_;
  bool has_twist_;
  bool has_collision_at_now_;
  rclcpp::Time stuck_started_since_;

  constexpr static float EPSILON = 1e-6;

  diagnostic_updater::Updater diag_updater_;

public:
  SafetyLimiterNode(const rclcpp::NodeOptions& options) : Node("safety_limiter", options)
    , last_cloud_stamp_(0L, RCL_ROS_TIME)
    , cloud_accum_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_clear_(false)
    , last_disable_cmd_(0L, RCL_ROS_TIME)
    , hold_(0, 0)
    , hold_off_(0L, RCL_ROS_TIME)
    , watchdog_interval_(0, 0)
    , watchdog_stop_(false)
    , has_cloud_(false)
    , has_twist_(true)
    , has_collision_at_now_(false)
    , stuck_started_since_(rclcpp::Time(0L, RCL_ROS_TIME))
    , diag_updater_(this)
  {
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(1).transient_local());
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud>("collision", rclcpp::QoS(1).transient_local());
    pub_status_ = this->create_publisher<safety_limiter_msgs::msg::SafetyLimiterStatus>("~/status", rclcpp::QoS(1).transient_local());
    using std::placeholders::_1;
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_in",
        1, [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg){ cbTwist(msg); });
    sub_disable_ = this->create_subscription<std_msgs::msg::Bool>(
        "disable_safety",
        1, [this](const std_msgs::msg::Bool::ConstSharedPtr msg){ cbDisable(msg); });
    sub_watchdog_ = this->create_subscription<std_msgs::msg::Empty>(
        "watchdog_reset",
        1, [this](const std_msgs::msg::Empty::ConstSharedPtr msg){ cbWatchdogReset(msg); });

    param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    if (params_.num_input_clouds == 1)
    {
      sub_clouds_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "cloud",
          1, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ cbCloud(msg); }));
    }
    else
    {
      for (int i = 0; i < params_.num_input_clouds; ++i)
      {
        sub_clouds_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud" + std::to_string(i), 1, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ cbCloud(msg); }));
      }
    }

    watchdog_interval_ = rclcpp::Duration::from_seconds(params_.watchdog_interval);
    max_values_[0] = std::numeric_limits<double>::infinity();
    max_values_[1] = std::numeric_limits<double>::infinity();

    std::regex pattern(R"(\[\s*(-?[\d\.]+)\s*,\s*(-?[\d\.]+)\s*\])");
    auto begin = std::sregex_iterator(params_.footprint.begin(), params_.footprint.end(), pattern);
    auto end = std::sregex_iterator();

    footprint_radius_ = 0;
    for (auto it = begin; it != end; it++)
    {
      vec v;
      v[0] = std::stod((*it)[1].str());
      v[1] = std::stod((*it)[2].str());
      footprint_p.v.push_back(v);

      const float dist = std::hypot(v[0], v[1]);
      if (dist > footprint_radius_)
        footprint_radius_ = dist;
    }

    if (footprint_p.v.size() < 3)
    {
      RCLCPP_FATAL(this->get_logger(), "Invalid footprint");
      throw std::runtime_error("Invalid footprint");
    }
    footprint_p.v.push_back(footprint_p.v.front());
    RCLCPP_INFO(this->get_logger(), "footprint radius: %0.3f", footprint_radius_);

    diag_updater_.setHardwareID("none");
    diag_updater_.add("Collision", this, &SafetyLimiterNode::diagnoseCollision);

    tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

    cbParameter(params_);

    param_listener_->setUserCallback([this](const Params& params){ cbParameter(params); });

    predict_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / params_.freq), std::bind(&SafetyLimiterNode::cbPredictTimer, this));

    if (watchdog_interval_ != rclcpp::Duration::from_seconds(0.0))
    {
      watchdog_timer_ =
          this->create_wall_timer(watchdog_interval_.to_chrono<std::chrono::duration<double>>(), std::bind(&SafetyLimiterNode::cbWatchdogTimer, this));
    }
  }

protected:
  void cbWatchdogReset(const std_msgs::msg::Empty::ConstSharedPtr /*msg*/)
  {
    watchdog_timer_ = this->create_wall_timer(watchdog_interval_.to_chrono<std::chrono::duration<double>>(), std::bind(&SafetyLimiterNode::cbWatchdogTimer, this));
    watchdog_stop_ = false;
  }
  void cbWatchdogTimer()
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Watchdog timed-out");
    watchdog_stop_ = true;
    r_lim_ = 0;
    geometry_msgs::msg::Twist cmd_vel;
    pub_twist_->publish(cmd_vel);

    diag_updater_.force_update();
  }
  void cbPredictTimer()
  {
    if (!has_twist_)
      return;
    if (!has_cloud_)
      return;

    if (this->now() - last_cloud_stamp_ > rclcpp::Duration::from_seconds(params_.cloud_timeout))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: PointCloud timed-out");
      geometry_msgs::msg::Twist cmd_vel;
      pub_twist_->publish(cmd_vel);

      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      has_cloud_ = false;
      r_lim_ = 0;

      diag_updater_.force_update();
      return;
    }

    rclcpp::Time now = this->now();
    const double r_lim_current = predict(twist_);

    if (r_lim_current < r_lim_)
      r_lim_ = r_lim_current;

    if (r_lim_current < 1.0)
      hold_off_ = now + hold_;

    cloud_clear_ = true;

    diag_updater_.force_update();
  }
  void cbParameter(const Params& params)
  {
    params_ = params;
    vel_[0] = params_.lin_vel;
    acc_[0] = params_.lin_acc;
    vel_[1] = params_.ang_vel;
    acc_[1] = params_.ang_acc;
    max_values_[0] = params_.max_linear_vel;
    max_values_[1] = params_.max_angular_vel;
    z_range_[0] = params_.z_range_min;
    z_range_[1] = params_.z_range_max;
    hold_ = rclcpp::Duration::from_seconds(std::max(params_.hold, 1.0 / params_.freq));

    tmax_ = 0.0;
    for (int i = 0; i < 2; i++)
    {
      auto t = vel_[i] / acc_[i];
      if (tmax_ < t)
        tmax_ = t;
    }
    tmax_ *= 1.5;
    tmax_ += std::max(params_.d_margin / vel_[0], params_.yaw_margin / vel_[1]);
    r_lim_ = 1.0;
  }
  double predict(const geometry_msgs::msg::Twist& /*in*/)
  {
    if (cloud_accum_->size() == 0)
    {
      if (params_.allow_empty_cloud)
      {
        return 1.0;
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Empty pointcloud passed.");
      return 0.0;
    }

    const bool can_transform = tfbuf_->canTransform(
        params_.base_frame, cloud_accum_->header.frame_id,
        pcl_conversions::fromPCL(cloud_accum_->header.stamp));
    const rclcpp::Time stamp =
        can_transform ? pcl_conversions::fromPCL(cloud_accum_->header.stamp) : rclcpp::Time(0L, RCL_ROS_TIME);

    geometry_msgs::msg::TransformStamped fixed_to_base;
    try
    {
      fixed_to_base = tfbuf_->lookupTransform(
          params_.base_frame, cloud_accum_->header.frame_id, stamp);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Transform failed: %s", e.what());
      return 0.0;
    }

    const Eigen::Affine3f fixed_to_base_eigen =
        Eigen::Translation3f(
            fixed_to_base.transform.translation.x,
            fixed_to_base.transform.translation.y,
            fixed_to_base.transform.translation.z) *
        Eigen::Quaternionf(
            fixed_to_base.transform.rotation.w,
            fixed_to_base.transform.rotation.x,
            fixed_to_base.transform.rotation.y,
            fixed_to_base.transform.rotation.z);
    pcl::transformPointCloud(*cloud_accum_, *cloud_accum_, fixed_to_base_eigen);

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> ds;
    ds.setInputCloud(cloud_accum_);
    ds.setLeafSize(params_.downsample_grid, params_.downsample_grid, params_.downsample_grid);
    ds.filter(*pc);

    auto filter_z = [this](pcl::PointXYZ& p)
    {
      if (p.z < this->z_range_[0] || this->z_range_[1] < p.z)
        return true;
      p.z = 0.0;
      return false;
    };
    pc->erase(std::remove_if(pc->points.begin(), pc->points.end(), filter_z),
              pc->points.end());

    if (pc->size() == 0)
    {
      if (params_.allow_empty_cloud)
      {
        return 1.0;
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Empty pointcloud passed.");
      return 0.0;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pc);

    Eigen::Affine3f move;
    Eigen::Affine3f move_inv;
    Eigen::Affine3f motion =
        Eigen::AngleAxisf(-twist_.angular.z * params_.dt, Eigen::Vector3f::UnitZ()) *
        Eigen::Translation3f(Eigen::Vector3f(-twist_.linear.x * params_.dt, -twist_.linear.y * params_.dt, 0.0));
    Eigen::Affine3f motion_inv =
        Eigen::Translation3f(Eigen::Vector3f(twist_.linear.x * params_.dt, twist_.linear.y * params_.dt, 0.0)) *
        Eigen::AngleAxisf(twist_.angular.z * params_.dt, Eigen::Vector3f::UnitZ());
    move.setIdentity();
    move_inv.setIdentity();
    sensor_msgs::msg::PointCloud col_points;
    col_points.header.frame_id = params_.base_frame;
    col_points.header.stamp = this->now();

    float d_col = 0;
    float yaw_col = 0;
    bool has_collision = false;
    float d_escape_remain = 0;
    float yaw_escape_remain = 0;
    has_collision_at_now_ = false;
    const double linear_vel = std::hypot(twist_.linear.x, twist_.linear.y);

    for (float t = 0; t < tmax_; t += params_.dt)
    {
      if (t != 0)
      {
        d_col += linear_vel * params_.dt;
        d_escape_remain -= linear_vel * params_.dt;
        yaw_col += twist_.angular.z * params_.dt;
        yaw_escape_remain -= std::abs(twist_.angular.z) * params_.dt;
        move = move * motion;
        move_inv = move_inv * motion_inv;
      }

      pcl::PointXYZ center;
      center = pcl::transformPoint(center, move_inv);
      std::vector<int> indices;
      std::vector<float> dist;
      const int num = kdtree.radiusSearch(center, footprint_radius_, indices, dist);
      if (num == 0)
        continue;

      bool colliding = false;
      for (auto& i : indices)
      {
        auto& p = pc->points[i];
        auto point = pcl::transformPoint(p, move);
        vec v(point.x, point.y);
        if (footprint_p.inside(v))
        {
          geometry_msgs::msg::Point32 pos;
          pos.x = p.x;
          pos.y = p.y;
          pos.z = p.z;
          col_points.points.push_back(pos);
          colliding = true;
          break;
        }
      }
      if (colliding)
      {
        d_col -= linear_vel * params_.dt;
        yaw_col -= twist_.angular.z * params_.dt;
        if (t == 0)
        {
          // The robot is already in collision.
          // Allow movement under params_.d_escape and params_.yaw_escape
          d_escape_remain = params_.d_escape;
          yaw_escape_remain = params_.yaw_escape;
          has_collision_at_now_ = true;
        }
        if (d_escape_remain <= 0 || yaw_escape_remain <= 0)
        {
          if (has_collision_at_now_)
          {
            // It's not possible to escape from collision; stop completely.
            d_col = yaw_col = 0;
          }

          has_collision = true;
          break;
        }
      }
    }
    pub_cloud_->publish(col_points);

    if (has_collision_at_now_)
    {
      if (stuck_started_since_ == rclcpp::Time(0L, RCL_ROS_TIME))
        stuck_started_since_ = this->now();
    }
    else
    {
      if (stuck_started_since_ != rclcpp::Time(0L, RCL_ROS_TIME))
        stuck_started_since_ = rclcpp::Time(0L, RCL_ROS_TIME);
    }

    if (!has_collision)
      return 1.0;

    const float delay = 1.0 * (1.0 / params_.freq) + params_.dt;
    const float acc_dtsq[2] =
        {
            static_cast<float>(acc_[0] * std::pow(delay, 2)),
            static_cast<float>(acc_[1] * std::pow(delay, 2)),
        };

    d_col = std::max<float>(
        0.0,
        std::abs(d_col) - params_.d_margin + acc_dtsq[0] -
            std::sqrt(std::pow(acc_dtsq[0], 2) + 2 * acc_dtsq[0] * std::abs(d_col)));
    yaw_col = std::max<float>(
        0.0,
        std::abs(yaw_col) - params_.yaw_margin + acc_dtsq[1] -
            std::sqrt(std::pow(acc_dtsq[1], 2) + 2 * acc_dtsq[1] * std::abs(yaw_col)));

    float d_r =
        std::sqrt(std::abs(2 * acc_[0] * d_col)) / linear_vel;
    float yaw_r =
        std::sqrt(std::abs(2 * acc_[1] * yaw_col)) / std::abs(twist_.angular.z);
    if (!std::isfinite(d_r))
      d_r = 1.0;
    if (!std::isfinite(yaw_r))
      yaw_r = 1.0;

    return std::min(d_r, yaw_r);
  }

  geometry_msgs::msg::Twist
  limit(const geometry_msgs::msg::Twist& in)
  {
    auto out = in;
    if (r_lim_ < 1.0 - EPSILON)
    {
      out.linear.x *= r_lim_;
      out.linear.y *= r_lim_;
      out.angular.z *= r_lim_;
      if (std::abs(in.linear.x - out.linear.x) > EPSILON ||
          std::abs(in.linear.y - out.linear.y) > EPSILON ||
          std::abs(in.angular.z - out.angular.z) > EPSILON)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
            1000, "safety_limiter: (%0.2f, %0.2f, %0.2f)->(%0.2f, %0.2f, %0.2f)",
            in.linear.x, in.linear.y, in.angular.z,
            out.linear.x, out.linear.y, out.angular.z);
      }
    }
    return out;
  }

  geometry_msgs::msg::Twist
  limitMaxVelocities(const geometry_msgs::msg::Twist& in)
  {
    auto out = in;
    if (max_values_[0] <= 0.0)
    {
      out.linear.x = 0;
      out.linear.y = 0;
    }
    else
    {
      const double out_linear_vel = std::hypot(out.linear.x, out.linear.y);
      if (out_linear_vel > max_values_[0])
      {
        const double vel_ratio = max_values_[0] / out_linear_vel;
        out.linear.x *= vel_ratio;
        out.linear.y *= vel_ratio;
      }
    }
    out.angular.z = (out.angular.z > 0) ?
                        std::min(out.angular.z, max_values_[1]) :
                        std::max(out.angular.z, -max_values_[1]);

    return out;
  }

  polygon footprint_p;

  void cbTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    rclcpp::Time now = this->now();

    twist_ = *msg;
    has_twist_ = true;

    if (now - last_disable_cmd_ < rclcpp::Duration::from_seconds(params_.disable_timeout))
    {
      pub_twist_->publish(limitMaxVelocities(twist_));
    }
    else if (!has_cloud_ || watchdog_stop_)
    {
      geometry_msgs::msg::Twist cmd_vel;
      pub_twist_->publish(cmd_vel);
    }
    else
    {
      geometry_msgs::msg::Twist cmd_vel = limitMaxVelocities(limit(twist_));
      pub_twist_->publish(cmd_vel);

      if (now > hold_off_)
        r_lim_ = 1.0;
    }
  }

  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const bool can_transform = tfbuf_->canTransform(
        params_.fixed_frame, msg->header.frame_id, msg->header.stamp);
    const rclcpp::Time stamp =
        can_transform ? rclcpp::Time(msg->header.stamp) : rclcpp::Time(0L, RCL_ROS_TIME);

    sensor_msgs::msg::PointCloud2 cloud_msg_fixed;
    try
    {
      const geometry_msgs::msg::TransformStamped cloud_to_fixed =
          tfbuf_->lookupTransform(params_.fixed_frame, msg->header.frame_id, stamp);
      tf2::doTransform(*msg, cloud_msg_fixed, cloud_to_fixed);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Transform failed: %s", e.what());
      return;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_fixed(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_fixed->header.frame_id = params_.fixed_frame;
    pcl::fromROSMsg(cloud_msg_fixed, *cloud_fixed);

    if (cloud_clear_)
    {
      cloud_clear_ = false;
      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    *cloud_accum_ += *cloud_fixed;
    cloud_accum_->header.frame_id = params_.fixed_frame;
    last_cloud_stamp_ = msg->header.stamp;
    has_cloud_ = true;
  }
  void cbDisable(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (msg->data)
    {
      last_disable_cmd_ = this->now();
    }
  }

  void diagnoseCollision(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    safety_limiter_msgs::msg::SafetyLimiterStatus status_msg;

    if (!has_cloud_ || watchdog_stop_)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Stopped due to data timeout.");
    }
    else if (r_lim_ == 1.0)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    }
    else if (r_lim_ < EPSILON)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   (has_collision_at_now_) ?
                       "Cannot escape from collision." :
                       "Trying to avoid collision, but cannot move anymore.");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   (has_collision_at_now_) ?
                       "Escaping from collision." :
                       "Reducing velocity to avoid collision.");
    }
    stat.addf("Velocity Limit Ratio", "%.2f", r_lim_);
    stat.add("Pointcloud Availability", has_cloud_ ? "true" : "false");
    stat.add("Watchdog Timeout", watchdog_stop_ ? "true" : "false");

    status_msg.limit_ratio = r_lim_;
    status_msg.is_cloud_available = has_cloud_;
    status_msg.has_watchdog_timed_out = watchdog_stop_;
    status_msg.stuck_started_since = stuck_started_since_;

    pub_status_->publish(status_msg);
  }
};

}  // namespace safety_limiter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(safety_limiter::SafetyLimiterNode)