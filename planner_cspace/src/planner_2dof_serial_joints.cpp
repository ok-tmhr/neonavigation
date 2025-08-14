/*
 * Copyright (c) 2014-2020, the neonavigation authors
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
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <omp.h>

#include <rclcpp/rclcpp.hpp>

#include <planner_cspace_msgs/msg/planner_status.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_2dof_serial_joints/grid_astar_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace planner_cspace
{
namespace planner_2dof_serial_joints
{
class Planner2dofSerialJointsNode : public rclcpp::Node
{
public:
  using Astar = GridAstar<2, 0>;
  using Ptr = std::shared_ptr<Planner2dofSerialJointsNode>;

private:
  rclcpp::Publisher<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;

  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  Astar as_;
  Astar::Gridmap<char, 0x40> cm_;
  GridAstarModel2DoFSerialJoint::Ptr model_;

  float freq_;
  float freq_min_;
  int resolution_;
  float avg_vel_;
  enum PointVelMode
  {
    VEL_PREV,
    VEL_NEXT,
    VEL_AVG
  };
  PointVelMode point_vel_;

  std::string group_;

  bool debug_aa_;

  class LinkBody
  {
  protected:
    class Vec3dof
    {
    public:
      float x_;
      float y_;
      float th_;

      float dist(const Vec3dof& b)
      {
        return std::hypot(b.x_ - x_, b.y_ - y_);
      }
    };

  public:
    float radius_[2];
    float vmax_;
    float length_;
    std::string name_;
    Vec3dof origin_;
    Vec3dof gain_;
    float current_th_;

    LinkBody()
    {
      gain_.x_ = 1.0;
      gain_.y_ = 1.0;
      gain_.th_ = 1.0;
    }
    Vec3dof end(const float th) const
    {
      Vec3dof e = origin_;
      e.x_ += cosf(e.th_ + th * gain_.th_) * length_;
      e.y_ += sinf(e.th_ + th * gain_.th_) * length_;
      e.th_ += th;
      return e;
    }
    bool isCollide(const LinkBody b, const float th0, const float th1)
    {
      auto end0 = end(th0);
      auto end1 = b.end(th1);
      auto& end0r = radius_[1];
      auto& end1r = b.radius_[1];
      auto& origin0 = origin_;
      auto& origin1 = b.origin_;
      auto& origin0r = radius_[0];
      auto& origin1r = b.radius_[0];

      if (end0.dist(end1) < end0r + end1r)
        return true;
      if (end0.dist(origin1) < end0r + origin1r)
        return true;
      if (end1.dist(origin0) < end1r + origin0r)
        return true;

      // add side collision

      return false;
    }
  };
  LinkBody links_[2];

  planner_cspace_msgs::msg::PlannerStatus status_;
  sensor_msgs::msg::JointState joint_;
  rclcpp::Time replan_prev_;
  rclcpp::Duration replan_interval_;
  bool has_joint_states_;

  void cbJoint(const sensor_msgs::msg::JointState::ConstPtr& msg)
  {
    int id[2] = {-1, -1};
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      if (msg->name[i].compare(links_[0].name_) == 0)
        id[0] = i;
      else if (msg->name[i].compare(links_[1].name_) == 0)
        id[1] = i;
    }
    if (id[0] == -1 || id[1] == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "joint_state does not contain link group %s.", group_.c_str());
      return;
    }
    links_[0].current_th_ = msg->position[id[0]];
    links_[1].current_th_ = msg->position[id[1]];
    has_joint_states_ = true;

    if ((replan_prev_ + replan_interval_ < now() ||
         replan_prev_ == rclcpp::Time(0LL, RCL_ROS_TIME)) &&
        replan_interval_ > rclcpp::Duration(0, 0))
    {
      replan();
    }
  }
  std::pair<rclcpp::Duration, std::pair<float, float>> cmd_prev_;
  trajectory_msgs::msg::JointTrajectory traj_prev_;
  int id_[2];
  void cbTrajectory(const trajectory_msgs::msg::JointTrajectory::ConstPtr& msg)
  {
    id_[0] = -1;
    id_[1] = -1;
    for (size_t i = 0; i < msg->joint_names.size(); i++)
    {
      if (msg->joint_names[i].compare(links_[0].name_) == 0)
        id_[0] = i;
      else if (msg->joint_names[i].compare(links_[1].name_) == 0)
        id_[1] = i;
    }
    if (id_[0] == -1 || id_[1] == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "joint_trajectory does not contains link group %s.", group_.c_str());
      return;
    }
    if (msg->points.size() != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "single trajectory point required.");
    }
    decltype(cmd_prev_) cmd{msg->points[0].time_from_start, {}};
    cmd.second.first = msg->points[0].positions[id_[0]];
    cmd.second.second = msg->points[0].positions[id_[1]];
    if (cmd_prev_ == cmd)
      return;
    cmd_prev_ = cmd;
    traj_prev_ = *msg;
    avg_vel_ = -1.0;

    replan();
  }
  void replan()
  {
    if (!has_joint_states_)
      return;

    replan_prev_ = now();
    if (id_[0] == -1 || id_[1] == -1)
      return;

    Astar::Vecf start(
        links_[0].current_th_,
        links_[1].current_th_);
    Astar::Vecf end(
        static_cast<float>(traj_prev_.points[0].positions[id_[0]]),
        static_cast<float>(traj_prev_.points[0].positions[id_[1]]));

    RCLCPP_INFO(this->get_logger(), "link %s: %0.3f, %0.3f", group_.c_str(),
             traj_prev_.points[0].positions[id_[0]],
             traj_prev_.points[0].positions[id_[1]]);

    status_.status = planner_cspace_msgs::msg::PlannerStatus::DOING;
    status_.error = planner_cspace_msgs::msg::PlannerStatus::GOING_WELL;

    RCLCPP_INFO(this->get_logger(), "Start searching");
    std::list<Astar::Vecf> path;
    if (makePlan(start, end, path))
    {
      RCLCPP_INFO(this->get_logger(), "Trajectory found");

      if (avg_vel_ < 0)
      {
        float pos_sum = 0;
        for (auto it = path.begin(); it != path.end(); it++)
        {
          auto it_next = it;
          it_next++;
          if (it_next != path.end())
          {
            float diff[2], diff_max;
            diff[0] = std::abs((*it_next)[0] - (*it)[0]);
            diff[1] = std::abs((*it_next)[1] - (*it)[1]);
            diff_max = std::max(diff[0], diff[1]);
            pos_sum += diff_max;
          }
        }
        if (rclcpp::Duration(traj_prev_.points[0].time_from_start) <= rclcpp::Duration(0, 0))
        {
          avg_vel_ = std::min(links_[0].vmax_, links_[1].vmax_);
        }
        else
        {
          avg_vel_ = pos_sum / rclcpp::Duration(traj_prev_.points[0].time_from_start).seconds();
          if (avg_vel_ > links_[0].vmax_)
            avg_vel_ = links_[0].vmax_;
          if (avg_vel_ > links_[1].vmax_)
            avg_vel_ = links_[1].vmax_;
        }
      }

      trajectory_msgs::msg::JointTrajectory out;
      out.header = traj_prev_.header;
      out.header.stamp = rclcpp::Time(0);
      out.joint_names.resize(2);
      out.joint_names[0] = links_[0].name_;
      out.joint_names[1] = links_[1].name_;
      float pos_sum = 0.0;
      for (auto it = path.begin(); it != path.end(); it++)
      {
        if (it == path.begin())
          continue;

        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions.resize(2);
        p.velocities.resize(2);

        auto it_prev = it;
        it_prev--;
        auto it_next = it;
        it_next++;

        float diff[2], diff_max;
        diff[0] = std::abs((*it)[0] - (*it_prev)[0]);
        diff[1] = std::abs((*it)[1] - (*it_prev)[1]);
        diff_max = std::max(diff[0], diff[1]);
        pos_sum += diff_max;

        if (it_next == path.end())
        {
          p.velocities[0] = 0.0;
          p.velocities[1] = 0.0;
        }
        else
        {
          float dir[2], dir_max;
          switch (point_vel_)
          {
            default:
            case VEL_PREV:
              dir[0] = ((*it)[0] - (*it_prev)[0]);
              dir[1] = ((*it)[1] - (*it_prev)[1]);
              break;
            case VEL_NEXT:
              dir[0] = ((*it_next)[0] - (*it)[0]);
              dir[1] = ((*it_next)[1] - (*it)[1]);
              break;
            case VEL_AVG:
              dir[0] = ((*it_next)[0] - (*it_prev)[0]);
              dir[1] = ((*it_next)[1] - (*it_prev)[1]);
              break;
          }
          dir_max = std::max(std::abs(dir[0]), std::abs(dir[1]));
          float t = dir_max / avg_vel_;

          p.velocities[0] = dir[0] / t;
          p.velocities[1] = dir[1] / t;
        }
        p.time_from_start = rclcpp::Duration::from_seconds(pos_sum / avg_vel_);
        p.positions[0] = (*it)[0];
        p.positions[1] = (*it)[1];
        out.points.push_back(p);
      }
      pub_trajectory_->publish(out);
    }
    else
    {
      trajectory_msgs::msg::JointTrajectory out;
      out.header = traj_prev_.header;
      out.header.stamp = rclcpp::Time(0);
      out.joint_names.resize(2);
      out.joint_names[0] = links_[0].name_;
      out.joint_names[1] = links_[1].name_;
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.resize(2);
      p.positions[0] = links_[0].current_th_;
      p.positions[1] = links_[1].current_th_;
      p.velocities.resize(2);
      out.points.push_back(p);
      pub_trajectory_->publish(out);

      RCLCPP_WARN(this->get_logger(), "Trajectory not found");
    }

    status_.header.stamp = now();
    pub_status_->publish(status_);
  }

public:
  explicit Planner2dofSerialJointsNode(const std::string group_name)
    : rclcpp::Node("planner_2dof_serial_joints")
    , has_joint_states_(false)
    , replan_interval_(rclcpp::Duration(0, 0))
    , cmd_prev_(rclcpp::Duration(0, 0), {})
  {

    group_ = group_name;

    pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory",
        rclcpp::QoS(1).transient_local());
    using std::placeholders::_1;
    sub_trajectory_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "trajectory_in",
        1, std::bind(&Planner2dofSerialJointsNode::cbTrajectory, this, _1));
    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        1, std::bind(&Planner2dofSerialJointsNode::cbJoint, this, _1));

    pub_status_ = this->create_publisher<planner_cspace_msgs::msg::PlannerStatus>("~/" + group_ + "/status", rclcpp::QoS(1).transient_local());

    tfbuf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

    resolution_ = this->declare_parameter("resolution", 128);
    debug_aa_ = this->declare_parameter("debug_aa", false);

    double interval;
    interval = this->declare_parameter("replan_interval", 0.2);
    replan_interval_ = rclcpp::Duration::from_seconds(interval);
    replan_prev_ = rclcpp::Time(0);

    int queue_size_limit;
    queue_size_limit = this->declare_parameter("queue_size_limit", 0);
    as_.setQueueSizeLimit(queue_size_limit);

    status_.status = planner_cspace_msgs::msg::PlannerStatus::DONE;

    cm_.reset(Astar::Vec(resolution_ * 2, resolution_ * 2));
    as_.reset(Astar::Vec(resolution_ * 2, resolution_ * 2));
    cm_.clear(0);

    links_[0].name_ = this->declare_parameter(group_ + ".link0_name", std::string("link0"));
    links_[0].radius_[0] = this->declare_parameter(group_ + ".link0_joint_radius", 0.07f);
    links_[0].radius_[1] = this->declare_parameter(group_ + ".link0_end_radius", 0.07f);
    links_[0].length_ = this->declare_parameter(group_ + ".link0_length", 0.135f);
    links_[0].origin_.x_ = this->declare_parameter(group_ + ".link0_x", 0.22f);
    links_[0].origin_.y_ = this->declare_parameter(group_ + ".link0_y", 0.0f);
    links_[0].origin_.th_ = this->declare_parameter(group_ + ".link0_th", 0.0f);
    links_[0].gain_.th_ = this->declare_parameter(group_ + ".link0_gain_th", -1.0f);
    links_[0].vmax_ = this->declare_parameter(group_ + ".link0_vmax", 0.5f);
    links_[1].name_ = this->declare_parameter(group_ + ".link1_name", std::string("link1"));
    links_[1].radius_[0] = this->declare_parameter(group_ + ".link1_joint_radius", 0.07f);
    links_[1].radius_[1] = this->declare_parameter(group_ + ".link1_end_radius", 0.07f);
    links_[1].length_ = this->declare_parameter(group_ + ".link1_length", 0.27f);
    links_[1].origin_.x_ = this->declare_parameter(group_ + ".link1_x", -0.22f);
    links_[1].origin_.y_ = this->declare_parameter(group_ + ".link1_y", 0.0f);
    links_[1].origin_.th_ = this->declare_parameter(group_ + ".link1_th", 0.0f);
    links_[1].gain_.th_ = this->declare_parameter(group_ + ".link1_gain_th", 1.0f);
    links_[1].vmax_ = this->declare_parameter(group_ + ".link1_vmax", 0.5f);

    links_[0].current_th_ = 0.0;
    links_[1].current_th_ = 0.0;
    id_[0] = -1;
    id_[1] = -1;

    RCLCPP_INFO(this->get_logger(), "link group: %s", group_.c_str());
    RCLCPP_INFO(this->get_logger(), " - link0: %s", links_[0].name_.c_str());
    RCLCPP_INFO(this->get_logger(), " - link1: %s", links_[1].name_.c_str());

    Astar::Vecf euclid_cost_coef;
    euclid_cost_coef[0] = this->declare_parameter(group_ + ".link0_coef", 1.0f);
    euclid_cost_coef[1] = this->declare_parameter(group_ + ".link1_coef", 1.5f);

    CostCoeff cc;
    cc.weight_cost_ = this->declare_parameter(group_ + ".weight_cost", 4.0f);
    cc.expand_ = this->declare_parameter(group_ + ".expand", 0.1f);

    std::string point_vel_mode;
    point_vel_mode = this->declare_parameter(group_ + ".point_vel_mode", std::string("prev"));
    std::transform(point_vel_mode.begin(), point_vel_mode.end(), point_vel_mode.begin(), ::tolower);
    if (point_vel_mode.compare("prev") == 0)
      point_vel_ = VEL_PREV;
    else if (point_vel_mode.compare("next") == 0)
      point_vel_ = VEL_NEXT;
    else if (point_vel_mode.compare("avg") == 0)
      point_vel_ = VEL_AVG;
    else
      RCLCPP_ERROR(this->get_logger(), "point_vel_mode must be prev/next/avg");

    RCLCPP_INFO(this->get_logger(), "Resolution: %d", resolution_);
    Astar::Vec p;
    for (p[0] = 0; p[0] < resolution_ * 2; p[0]++)
    {
      for (p[1] = 0; p[1] < resolution_ * 2; p[1]++)
      {
        Astar::Vecf pf;
        grid2Metric(p, pf);

        if (links_[0].isCollide(links_[1], pf[0], pf[1]))
          cm_[p] = 100;
        // else if(pf[0] > M_PI || pf[1] > M_PI)
        //   cm_[p] = 50;
        else
          cm_[p] = 0;
      }
    }
    for (p[0] = 0; p[0] < resolution_ * 2; p[0]++)
    {
      for (p[1] = 0; p[1] < resolution_ * 2; p[1]++)
      {
        if (cm_[p] != 100)
          continue;

        Astar::Vec d;
        int range = std::lround(cc.expand_ * resolution_ / (2.0 * M_PI));
        for (d[0] = -range; d[0] <= range; d[0]++)
        {
          for (d[1] = -range; d[1] <= range; d[1]++)
          {
            Astar::Vec p2 = p + d;
            if ((unsigned int)p2[0] >= (unsigned int)resolution_ * 2 ||
                (unsigned int)p2[1] >= (unsigned int)resolution_ * 2)
              continue;
            int dist = std::max(std::abs(d[0]), abs(d[1]));
            int c = std::floor(100.0 * (range - dist) / range);
            if (cm_[p2] < c)
              cm_[p2] = c;
          }
        }
      }
    }

    int range;
    range = this->declare_parameter(group_ + ".range", 8);

    model_.reset(new GridAstarModel2DoFSerialJoint(
        euclid_cost_coef,
        resolution_,
        cm_,
        cc,
        range));

    int num_threads;
    num_threads = this->declare_parameter(group_ + ".num_threads", 1);
    omp_set_num_threads(num_threads);
  }

private:
  void grid2Metric(
      const int t0, const int t1,
      float& gt0, float& gt1)
  {
    gt0 = (t0 - resolution_) * 2.0 * M_PI / static_cast<float>(resolution_);
    gt1 = (t1 - resolution_) * 2.0 * M_PI / static_cast<float>(resolution_);
  }
  void metric2Grid(
      int& t0, int& t1,
      const float gt0, const float gt1)
  {
    t0 = std::lround(gt0 * resolution_ / (2.0 * M_PI)) + resolution_;
    t1 = std::lround(gt1 * resolution_ / (2.0 * M_PI)) + resolution_;
  }
  void grid2Metric(
      const Astar::Vec t,
      Astar::Vecf& gt)
  {
    grid2Metric(t[0], t[1], gt[0], gt[1]);
  }
  void metric2Grid(
      Astar::Vec& t,
      const Astar::Vecf gt)
  {
    metric2Grid(t[0], t[1], gt[0], gt[1]);
  }
  bool makePlan(const Astar::Vecf sg, const Astar::Vecf eg, std::list<Astar::Vecf>& path)
  {
    Astar::Vec s, e;
    metric2Grid(s, sg);
    metric2Grid(e, eg);
    RCLCPP_INFO(this->get_logger(), "Planning from (%d, %d) to (%d, %d)",
             s[0], s[1], e[0], e[1]);

    if (cm_[s] == 100)
    {
      RCLCPP_WARN(this->get_logger(), "Path plan failed (current status is in collision)");
      status_.error = planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
      return false;
    }
    if (cm_[e] == 100)
    {
      RCLCPP_WARN(this->get_logger(), "Path plan failed (goal status is in collision)");
      status_.error = planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
      return false;
    }
    Astar::Vec d = e - s;
    d.cycle(resolution_, resolution_);

    std::vector<Astar::VecWithCost> starts;
    starts.emplace_back(s);

    if (model_->cost(s, e, starts, e) >= model_->euclidCost(d))
    {
      path.push_back(sg);
      path.push_back(eg);
      if (s == e)
      {
        replan_prev_ = rclcpp::Time(0);
      }
      return true;
    }
    std::list<Astar::Vec> path_grid;
    // const auto ts = std::chrono::high_resolution_clock::now();
    float cancel = std::numeric_limits<float>::max();
    if (replan_interval_ >= rclcpp::Duration(0, 0))
      cancel = replan_interval_.seconds();
    if (!as_.search(
            starts, e, path_grid, model_,
            std::bind(&Planner2dofSerialJointsNode::cbProgress, this, std::placeholders::_1, std::placeholders::_2),
            0, cancel, true))
    {
      RCLCPP_WARN(this->get_logger(), "Path plan failed (goal unreachable)");
      status_.error = planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
      return false;
    }
    // const auto tnow = std::chrono::high_resolution_clock::now();
    // RCLCPP_INFO(this->get_logger(), "Path found (%0.3f sec.)",
    //   std::chrono::duration<float>(tnow - ts).count());

    bool first = false;
    Astar::Vec n_prev = s;
    path.push_back(sg);
    int i = 0;
    for (auto& n : path_grid)
    {
      if (!first)
      {
        first = true;
        continue;
      }
      if (i == 0)
        RCLCPP_INFO(this->get_logger(), "  next: %d, %d", n[0], n[1]);
      Astar::Vec n_diff = n - n_prev;
      n_diff.cycle(resolution_, resolution_);
      Astar::Vec n2 = n_prev + n_diff;
      n_prev = n2;

      Astar::Vecf p;
      grid2Metric(n2, p);
      path.push_back(p);
      i++;
    }
    float prec = 2.0 * M_PI / static_cast<float>(resolution_);
    Astar::Vecf egp = eg;
    if (egp[0] < 0)
      egp[0] += std::ceil(-egp[0] / M_PI * 2.0) * M_PI * 2.0;
    if (egp[1] < 0)
      egp[1] += std::ceil(-egp[1] / M_PI * 2.0) * M_PI * 2.0;
    path.back()[0] += fmod(egp[0] + prec / 2.0, prec) - prec / 2.0;
    path.back()[1] += fmod(egp[1] + prec / 2.0, prec) - prec / 2.0;

    if (debug_aa_)
    {
      Astar::Vec p;
      for (p[0] = resolution_ / 2; p[0] < resolution_ * 3 / 2; p[0]++)
      {
        for (p[1] = resolution_ / 2; p[1] < resolution_ * 3 / 2; p[1]++)
        {
          bool found = false;
          for (auto& g : path_grid)
          {
            if (g == p)
              found = true;
          }
          if (p == s)
            printf("\033[31ms\033[0m");
          else if (p == e)
            printf("\033[31me\033[0m");
          else if (found)
            printf("\033[34m*\033[0m");
          else
            printf("%d", cm_[p] / 11);
        }
        printf("\n");
      }
      printf("\n");
    }

    return true;
  }
  bool cbProgress(const std::list<Astar::Vec>& /* path_grid */, const SearchStats& /* stats */)
  {
    return false;
  }
};
}  // namespace planner_2dof_serial_joints
}  // namespace planner_cspace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = rclcpp::Node::make_shared("planner_2dof_serial_joints");

  std::vector<planner_cspace::planner_2dof_serial_joints::Planner2dofSerialJointsNode::Ptr> jys;
  int n;
  n = node->declare_parameter("num_groups", 1);
  for (int i = 0; i < n; i++)
  {
    std::string name;
    name = node->declare_parameter("group" + std::to_string(i) + "_name",
              std::string("group") + std::to_string(i));
    auto jy = std::make_shared<planner_cspace::planner_2dof_serial_joints::Planner2dofSerialJointsNode>(name);
    jys.push_back(jy);
  }

  executor.add_node(node);
  for (auto &&jy : jys)
  {
    executor.add_node(jy);
  }

  executor.spin();

  return 0;
}
