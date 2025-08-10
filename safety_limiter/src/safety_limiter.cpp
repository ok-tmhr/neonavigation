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

#include <algorithm>
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
// #include <dynamic_reconfigure/server.h>
#include <geometry_msgs/msg/twist.hpp>
#include <safety_limiter_msgs/msg/safety_limiter_status.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>


// #include <safety_limiter/SafetyLimiterConfig.h>

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
// bool XmlRpc_isNumber(XmlRpc::XmlRpcValue& value)
// {
//   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
//          value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
// }

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
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  // boost::recursive_mutex parameter_server_mutex_;
  // std::unique_ptr<dynamic_reconfigure::Server<SafetyLimiterConfig>> parameter_server_;
  std::shared_ptr<rclcpp::ParameterEventHandler> event_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> callback_handles_;

  geometry_msgs::msg::Twist twist_;
  rclcpp::Time last_cloud_stamp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accum_;
  bool cloud_clear_;
  double hz_;
  double timeout_;
  double disable_timeout_;
  double vel_[2];
  double acc_[2];
  double tmax_;
  double dt_;
  double d_margin_;
  double d_escape_;
  double yaw_margin_;
  double yaw_escape_;
  double r_lim_;
  double max_values_[2];
  double z_range_[2];
  float footprint_radius_;
  double downsample_grid_;
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  rclcpp::Time last_disable_cmd_;
  rclcpp::Duration hold_;
  rclcpp::Time hold_off_;
  rclcpp::Duration watchdog_interval_;
  bool allow_empty_cloud_;

  bool watchdog_stop_;
  bool has_cloud_;
  bool has_twist_;
  bool has_collision_at_now_;
  rclcpp::Time stuck_started_since_;

  constexpr static float EPSILON = 1e-6;

  diagnostic_updater::Updater diag_updater_;

public:
  SafetyLimiterNode()
    : Node("safety_limiter")
    , tfbuf_(this->get_clock())
    , tfl_(tfbuf_)
    , cloud_accum_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_clear_(false)
    , last_disable_cmd_(0LL, RCL_ROS_TIME)
    , hold_(0, 0)
    , hold_off_(0LL, RCL_ROS_TIME)
    , watchdog_interval_(0, 0)
    , watchdog_stop_(false)
    , has_cloud_(false)
    , has_twist_(true)
    , has_collision_at_now_(false)
    , stuck_started_since_(rclcpp::Time(0LL, RCL_ROS_TIME))
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
        1, std::bind(&SafetyLimiterNode::cbTwist, this, _1));
    sub_disable_ = this->create_subscription<std_msgs::msg::Bool>(
        "disable_safety",
        1, std::bind(&SafetyLimiterNode::cbDisable, this, _1));
    sub_watchdog_ = this->create_subscription<std_msgs::msg::Empty>(
        "watchdog_reset",
        1, std::bind(&SafetyLimiterNode::cbWatchdogReset, this, _1));

    int num_input_clouds;
    num_input_clouds = this->declare_parameter("num_input_clouds", 1);
    if (num_input_clouds == 1)
    {
      sub_clouds_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "cloud",
          1, std::bind(&SafetyLimiterNode::cbCloud, this, _1)));
    }
    else
    {
      for (int i = 0; i < num_input_clouds; ++i)
      {
        sub_clouds_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud" + std::to_string(i), 1, std::bind(&SafetyLimiterNode::cbCloud, this, _1)));
      }
    }

    if (this->has_parameter("t_margin"))
      RCLCPP_WARN(this->get_logger(), "safety_limiter: t_margin parameter is obsolated. Use d_margin and yaw_margin instead.");
    base_frame_id_ = this->declare_parameter("base_frame", std::string("base_link"));
    fixed_frame_id_ = this->declare_parameter("fixed_frame", std::string("odom"));
    double watchdog_interval_d;
    watchdog_interval_d = this->declare_parameter("watchdog_interval", 0.0);
    watchdog_interval_ = rclcpp::Duration::from_seconds(watchdog_interval_d);

    const auto desc = [](const double from, const double to)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.floating_point_range.resize(1);
      desc.floating_point_range[0].from_value = from;
      desc.floating_point_range[0].to_value = to;
      return desc;
    };

    hz_ = this->declare_parameter("freq", 6.0, desc(0.0, 100.0));
    timeout_ = this->declare_parameter("cloud_timeout", 0.8, desc(0.0, 10.0));
    disable_timeout_ = this->declare_parameter("disable_timeout", 0.1, desc(0.0, 10.0));
    vel_[0] = this->declare_parameter("lin_vel", 0.5, desc(0.0, 10.0));
    acc_[0] = this->declare_parameter("lin_acc", 1.0, desc(0.0, 10.0));
    max_values_[0] = this->declare_parameter("max_linear_vel", 10.0, desc(0.0, 10.0));
    vel_[1] = this->declare_parameter("ang_vel", 0.8, desc(0.0, 10.0));
    acc_[1] = this->declare_parameter("ang_acc", 1.6, desc(0.0, 10.0));
    max_values_[1] = this->declare_parameter("max_angular_vel", 10.0, desc(0.0, 10.0));
    z_range_[0] = this->declare_parameter("z_range_min", 0.0, desc(-3.0, 3.0));
    z_range_[1] = this->declare_parameter("z_range_max", 0.5, desc(-3.0, 3.0));
    dt_ = this->declare_parameter("dt", 0.1, desc(0.0, 1.0));
    d_margin_ = this->declare_parameter("d_margin", 0.2, desc(0.0, 1.0));
    d_escape_ = this->declare_parameter("d_escape", 0.05, desc(0.0, 1.0));
    yaw_margin_ = this->declare_parameter("yaw_margin", 0.2, desc(0.0, 1.57));
    yaw_escape_ = this->declare_parameter("yaw_escape", 0.05, desc(0.0, 1.57));
    downsample_grid_ = this->declare_parameter("downsample_grid", 0.05, desc(0.0, 1.0));
    auto hold = this->declare_parameter("hold", 0.0, desc(0.0, 10.0));
    hold_ = rclcpp::Duration::from_seconds(std::max(hold, 1.0 / hz_));
    allow_empty_cloud_ = this->declare_parameter("allow_empty_cloud", false);

    cbParameter();

    std::string footprint = this->declare_parameter("footprint", "");
    if (!this->has_parameter("footprint"))
    {
      RCLCPP_FATAL(this->get_logger(), "Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified");
    }
    this->get_parameter("footprint", footprint);
    std::regex pattern(R"(\[\s*(-?[\d\.]+)\s*,\s*(-?[\d\.]+)\s*\])");

    auto begin = std::sregex_iterator(footprint.begin(), footprint.end(), pattern);
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

    event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    std::map<std::string, std::function<void(const rclcpp::Parameter&)>> callback_map = {
      {"freq", [this](const rclcpp::Parameter& p){ hz_ = p.as_double(); cbParameter(); }},
      {"cloud_timeout", [this](const rclcpp::Parameter& p){ timeout_ = p.as_double(); cbParameter(); }},
      {"disable_timeout", [this](const rclcpp::Parameter& p){ disable_timeout_ = p.as_double(); cbParameter(); }},
      {"lin_vel", [this](const rclcpp::Parameter& p){ vel_[0] = p.as_double(); cbParameter(); }},
      {"lin_acc", [this](const rclcpp::Parameter& p){ acc_[0] = p.as_double(); cbParameter(); }},
      {"max_linear_vel", [this](const rclcpp::Parameter& p){ max_values_[0] = p.as_double(); cbParameter(); }},
      {"ang_vel", [this](const rclcpp::Parameter& p){ vel_[1] = p.as_double(); cbParameter(); }},
      {"ang_acc", [this](const rclcpp::Parameter& p){ acc_[1] = p.as_double(); cbParameter(); }},
      {"max_angular_vel", [this](const rclcpp::Parameter& p){ max_values_[1] = p.as_double(); cbParameter(); }},
      {"z_range_min", [this](const rclcpp::Parameter& p){ z_range_[0] = p.as_double(); cbParameter(); }},
      {"z_range_max", [this](const rclcpp::Parameter& p){ z_range_[1] = p.as_double(); cbParameter(); }},
      {"dt", [this](const rclcpp::Parameter& p){ dt_ = p.as_double(); cbParameter(); }},
      {"d_margin", [this](const rclcpp::Parameter& p){ d_margin_ = p.as_double(); cbParameter(); }},
      {"d_escape", [this](const rclcpp::Parameter& p){ d_escape_ = p.as_double(); cbParameter(); }},
      {"yaw_margin", [this](const rclcpp::Parameter& p){ yaw_margin_ = p.as_double(); cbParameter(); }},
      {"yaw_escape", [this](const rclcpp::Parameter& p){ yaw_escape_ = p.as_double(); cbParameter(); }},
      {"downsample_grid", [this](const rclcpp::Parameter& p){ downsample_grid_ = p.as_double(); cbParameter(); }},
      {"hold", [this](const rclcpp::Parameter& p){ hold_ = rclcpp::Duration::from_seconds(std::max(p.as_double(), 1.0 / hz_)); cbParameter(); }},
      {"allow_empty_cloud", [this](const rclcpp::Parameter& p){ allow_empty_cloud_ = p.as_bool(); cbParameter(); }},
    };

    for (const auto& [name, cb] : callback_map)
    {
      callback_handles_.push_back(event_handler_->add_parameter_callback(name, cb));
    }
  }
  void spin()
  {
    rclcpp::TimerBase::SharedPtr predict_timer =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / hz_), std::bind(&SafetyLimiterNode::cbPredictTimer, this));

    if (watchdog_interval_ != rclcpp::Duration::from_seconds(0.0))
    {
      watchdog_timer_ =
          this->create_wall_timer(watchdog_interval_.to_chrono<std::chrono::duration<double>>(), std::bind(&SafetyLimiterNode::cbWatchdogTimer, this));
    }

    rclcpp::spin(shared_from_this());
  }

protected:
  void cbWatchdogReset(const std_msgs::msg::Empty::ConstPtr& msg)
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

    if (this->now() - last_cloud_stamp_ > rclcpp::Duration::from_seconds(timeout_))
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
  void cbParameter()
  {
    RCLCPP_INFO(this->get_logger(), "call cbParameter");
    tmax_ = 0.0;
    for (int i = 0; i < 2; i++)
    {
      auto t = vel_[i] / acc_[i];
      if (tmax_ < t)
        tmax_ = t;
    }
    tmax_ *= 1.5;
    tmax_ += std::max(d_margin_ / vel_[0], yaw_margin_ / vel_[1]);
    r_lim_ = 1.0;
  }
  double predict(const geometry_msgs::msg::Twist& in)
  {
    if (cloud_accum_->size() == 0)
    {
      if (allow_empty_cloud_)
      {
        return 1.0;
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Empty pointcloud passed.");
      return 0.0;
    }

    const bool can_transform = tfbuf_.canTransform(
        base_frame_id_, cloud_accum_->header.frame_id,
        pcl_conversions::fromPCL(cloud_accum_->header.stamp));
    const rclcpp::Time stamp =
        can_transform ? pcl_conversions::fromPCL(cloud_accum_->header.stamp) : rclcpp::Time(0LL, RCL_ROS_TIME);

    geometry_msgs::msg::TransformStamped fixed_to_base;
    try
    {
      fixed_to_base = tfbuf_.lookupTransform(
          base_frame_id_, cloud_accum_->header.frame_id, stamp);
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> ds;
    ds.setInputCloud(cloud_accum_);
    ds.setLeafSize(downsample_grid_, downsample_grid_, downsample_grid_);
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
      if (allow_empty_cloud_)
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
        Eigen::AngleAxisf(-twist_.angular.z * dt_, Eigen::Vector3f::UnitZ()) *
        Eigen::Translation3f(Eigen::Vector3f(-twist_.linear.x * dt_, -twist_.linear.y * dt_, 0.0));
    Eigen::Affine3f motion_inv =
        Eigen::Translation3f(Eigen::Vector3f(twist_.linear.x * dt_, twist_.linear.y * dt_, 0.0)) *
        Eigen::AngleAxisf(twist_.angular.z * dt_, Eigen::Vector3f::UnitZ());
    move.setIdentity();
    move_inv.setIdentity();
    sensor_msgs::msg::PointCloud col_points;
    col_points.header.frame_id = base_frame_id_;
    col_points.header.stamp = this->now();

    float d_col = 0;
    float yaw_col = 0;
    bool has_collision = false;
    float d_escape_remain = 0;
    float yaw_escape_remain = 0;
    has_collision_at_now_ = false;
    const double linear_vel = std::hypot(twist_.linear.x, twist_.linear.y);

    for (float t = 0; t < tmax_; t += dt_)
    {
      if (t != 0)
      {
        d_col += linear_vel * dt_;
        d_escape_remain -= linear_vel * dt_;
        yaw_col += twist_.angular.z * dt_;
        yaw_escape_remain -= std::abs(twist_.angular.z) * dt_;
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
        d_col -= linear_vel * dt_;
        yaw_col -= twist_.angular.z * dt_;
        if (t == 0)
        {
          // The robot is already in collision.
          // Allow movement under d_escape_ and yaw_escape_
          d_escape_remain = d_escape_;
          yaw_escape_remain = yaw_escape_;
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
      if (stuck_started_since_ == rclcpp::Time(0LL, RCL_ROS_TIME))
        stuck_started_since_ = this->now();
    }
    else
    {
      if (stuck_started_since_ != rclcpp::Time(0LL, RCL_ROS_TIME))
        stuck_started_since_ = rclcpp::Time(0LL, RCL_ROS_TIME);
    }

    if (!has_collision)
      return 1.0;

    // delay compensation:
    //   solve for d_compensated: d_compensated = d - delay * sqrt(2 * acc * d_compensated)
    //     d_compensated = d + acc * delay^2 - sqrt((acc * delay^2)^2 + 2 * d * acc * delay^2)

    const float delay = 1.0 * (1.0 / hz_) + dt_;
    const float acc_dtsq[2] =
        {
            static_cast<float>(acc_[0] * std::pow(delay, 2)),
            static_cast<float>(acc_[1] * std::pow(delay, 2)),
        };

    d_col = std::max<float>(
        0.0,
        std::abs(d_col) - d_margin_ + acc_dtsq[0] -
            std::sqrt(std::pow(acc_dtsq[0], 2) + 2 * acc_dtsq[0] * std::abs(d_col)));
    yaw_col = std::max<float>(
        0.0,
        std::abs(yaw_col) - yaw_margin_ + acc_dtsq[1] -
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

  class vec
  {
  public:
    float c[2];
    vec(const float x, const float y)
    {
      c[0] = x;
      c[1] = y;
    }
    vec()
    {
      c[0] = c[1] = 0.0;
    }
    float& operator[](const int& i)
    {
      assert(i < 2);
      return c[i];
    }
    const float& operator[](const int& i) const
    {
      assert(i < 2);
      return c[i];
    }
    vec operator-(const vec& a) const
    {
      vec out = *this;
      out[0] -= a[0];
      out[1] -= a[1];
      return out;
    }
    float cross(const vec& a) const
    {
      return (*this)[0] * a[1] - (*this)[1] * a[0];
    }
    float dot(const vec& a) const
    {
      return (*this)[0] * a[0] + (*this)[1] * a[1];
    }
    float dist(const vec& a) const
    {
      return std::hypot((*this)[0] - a[0], (*this)[1] - a[1]);
    }
    float dist_line(const vec& a, const vec& b) const
    {
      return (b - a).cross((*this) - a) / b.dist(a);
    }
    float dist_linestrip(const vec& a, const vec& b) const
    {
      if ((b - a).dot((*this) - a) <= 0)
        return this->dist(a);
      if ((a - b).dot((*this) - b) <= 0)
        return this->dist(b);
      return std::abs(this->dist_line(a, b));
    }
  };
  class polygon
  {
  public:
    std::vector<vec> v;
    void move(const float& x, const float& y, const float& yaw)
    {
      const float cos_v = cosf(yaw);
      const float sin_v = sinf(yaw);
      for (auto& p : v)
      {
        const auto tmp = p;
        p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
        p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
      }
    }
    bool inside(const vec& a) const
    {
      int cn = 0;
      for (size_t i = 0; i < v.size() - 1; i++)
      {
        auto& v1 = v[i];
        auto& v2 = v[i + 1];
        if ((v1[1] <= a[1] && a[1] < v2[1]) ||
            (v2[1] <= a[1] && a[1] < v1[1]))
        {
          float lx;
          lx = v1[0] + (v2[0] - v1[0]) * (a[1] - v1[1]) / (v2[1] - v1[1]);
          if (a[0] < lx)
            cn++;
        }
      }
      return ((cn & 1) == 1);
    }
    float dist(const vec& a) const
    {
      float dist = std::numeric_limits<float>::max();
      for (size_t i = 0; i < v.size() - 1; i++)
      {
        auto& v1 = v[i];
        auto& v2 = v[i + 1];
        auto d = a.dist_linestrip(v1, v2);
        if (d < dist)
          dist = d;
      }
      return dist;
    }
  };

  polygon footprint_p;

  void cbTwist(const geometry_msgs::msg::Twist::ConstPtr& msg)
  {
    rclcpp::Time now = this->now();

    twist_ = *msg;
    has_twist_ = true;

    if (now - last_disable_cmd_ < rclcpp::Duration::from_seconds(disable_timeout_))
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

  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstPtr& msg)
  {
    const bool can_transform = tfbuf_.canTransform(
        fixed_frame_id_, msg->header.frame_id, msg->header.stamp);
    const rclcpp::Time stamp =
        can_transform ? rclcpp::Time(msg->header.stamp) : rclcpp::Time(0LL, RCL_ROS_TIME);

    sensor_msgs::msg::PointCloud2 cloud_msg_fixed;
    try
    {
      const geometry_msgs::msg::TransformStamped cloud_to_fixed =
          tfbuf_.lookupTransform(fixed_frame_id_, msg->header.frame_id, stamp);
      tf2::doTransform(*msg, cloud_msg_fixed, cloud_to_fixed);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Transform failed: %s", e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fixed(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_fixed->header.frame_id = fixed_frame_id_;
    pcl::fromROSMsg(cloud_msg_fixed, *cloud_fixed);

    if (cloud_clear_)
    {
      cloud_clear_ = false;
      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    *cloud_accum_ += *cloud_fixed;
    cloud_accum_->header.frame_id = fixed_frame_id_;
    last_cloud_stamp_ = msg->header.stamp;
    has_cloud_ = true;
  }
  void cbDisable(const std_msgs::msg::Bool::ConstPtr& msg)
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto limiter = std::make_shared<safety_limiter::SafetyLimiterNode>();
  limiter->spin();

  return 0;
}
