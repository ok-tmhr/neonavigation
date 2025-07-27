/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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
#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_tracker_msgs/srv/change_path.hpp>
#include <trajectory_tracker_msgs/msg/trajectory_server_status.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>

#include <trajectory_tracker/filter.h>

class ServerNode : public rclcpp::Node
{
public:
  ServerNode();
  ~ServerNode();
  void spin();

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::TrajectoryServerStatus>::SharedPtr pub_status_;
  rclcpp::Service<trajectory_tracker_msgs::srv::ChangePath>::SharedPtr srv_change_path_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> srv_im_fb_;

  nav_msgs::msg::Path path_;
  std::string topic_path_;
  trajectory_tracker_msgs::srv::ChangePath::Request req_path_;
  double hz_;
  std::vector<uint8_t> buffer_;
  int serial_size_;
  double filter_step_;
  trajectory_tracker::Filter* lpf_[2];

  bool loadFile();
  void loadPath();
  void change(trajectory_tracker_msgs::srv::ChangePath::Request::SharedPtr req,
              trajectory_tracker_msgs::srv::ChangePath::Response::SharedPtr res);
  void processFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstPtr& feedback);
  void updateIM();
  enum
  {
    MENU_DELETE = 1,
    MENU_ADD = 2
  };
  int update_num_;
  int max_markers_;
};

ServerNode::ServerNode()
  : rclcpp::Node("trajectory_server")
  , buffer_(1024)
{
  topic_path_ = this->declare_parameter<std::string>("path", "path");
  req_path_.filename = this->declare_parameter<std::string>("file", "a.path");
  hz_ = this->declare_parameter<double>("hz", 5.0);
  filter_step_ = this->declare_parameter<double>("filter_step", 0.0);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(2).transient_local());
  pub_status_ = this->create_publisher<trajectory_tracker_msgs::msg::TrajectoryServerStatus>("~/status", 2);

  using std::placeholders::_1;
  using std::placeholders::_2;
  srv_change_path_ = this->create_service<trajectory_tracker_msgs::srv::ChangePath>(
    "change_path", std::bind(&ServerNode::change, this, _1, _2));
  srv_im_fb_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(get_namespace(), shared_from_this());
  update_num_ = 0;
  max_markers_ = 0;
}
ServerNode::~ServerNode()
{
}

bool ServerNode::loadFile()
{
  std::ifstream ifs(req_path_.filename.c_str());
  if (ifs.good())
  {
    ifs.seekg(0, ifs.end);
    serial_size_ = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    buffer_.resize(serial_size_);
    ifs.read(reinterpret_cast<char*>(buffer_.data()), serial_size_);

    return true;
  }
  return false;
}

void ServerNode::processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstPtr& feedback)
{
  int id = std::atoi(feedback->marker_name.c_str());
  switch (feedback->event_type)
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      path_.poses[id].pose = feedback->pose;
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
      path_.header.stamp = this->now();
      pub_path_->publish(path_);
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      switch (feedback->menu_entry_id)
      {
        case MENU_DELETE:
          path_.poses.erase(path_.poses.begin() + id);
          break;
        case MENU_ADD:
          path_.poses.insert(path_.poses.begin() + id, path_.poses[id]);
          break;
      }
      pub_path_->publish(path_);
      updateIM();
      break;
  }
}

void ServerNode::updateIM()
{
  visualization_msgs::msg::InteractiveMarkerUpdate viz;
  viz.type = viz.KEEP_ALIVE;
  viz.seq_num = update_num_++;
  viz.server_id = "Path";
  srv_im_fb_->clear();
  int i = 0;
  for (auto& p : path_.poses)
  {
    visualization_msgs::msg::InteractiveMarker mark;
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::InteractiveMarkerControl ctl;
    visualization_msgs::msg::MenuEntry menu;
    mark.header = path_.header;
    mark.pose = p.pose;
    mark.scale = 1.0;
    std::stringstream ss;
    ss << i++;
    mark.name = ss.str();
    marker.ns = "Path";
    marker.id = i;
    marker.scale.x = 0.2;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.color.a = 1;
    marker.type = marker.ARROW;

    ss << " ctl";
    ctl.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), M_PI / 2.0));
    ctl.interaction_mode = ctl.MOVE_ROTATE;
    ctl.orientation_mode = ctl.INHERIT;
    ctl.always_visible = true;
    ctl.markers.push_back(marker);
    ctl.name = ss.str();
    mark.controls.push_back(ctl);

    ss << " menu";
    ctl.interaction_mode = ctl.MENU;
    ctl.name = ss.str();
    marker.type = marker.SPHERE;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    ctl.markers[0] = marker;
    mark.controls.push_back(ctl);

    menu.id = MENU_DELETE;
    menu.parent_id = 0;
    menu.title = "Delete";
    menu.command_type = menu.FEEDBACK;
    mark.menu_entries.push_back(menu);
    menu.id = MENU_ADD;
    menu.parent_id = 0;
    menu.title = "Add";

    mark.menu_entries.push_back(menu);
    using std::placeholders::_1;
    srv_im_fb_->insert(mark, std::bind(&ServerNode::processFeedback, this, _1));
    viz.markers.push_back(mark);
  }
  srv_im_fb_->applyChanges();
}

void ServerNode::change(trajectory_tracker_msgs::srv::ChangePath::Request::SharedPtr req,
                        trajectory_tracker_msgs::srv::ChangePath::Response::SharedPtr res)
{
  req_path_ = *req;
  res->success = false;

  if (loadFile())
  {
    res->success = true;
    rclcpp::SerializedMessage serialized_msg(serial_size_);
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, buffer_.data(), serial_size_);
    rclcpp::Serialization<nav_msgs::msg::Path> serializer;
    serializer.deserialize_message(&serialized_msg, &path_);
    path_.header.stamp = this->now();
    if (filter_step_ > 0)
    {
      std::cout << filter_step_ << std::endl;
      lpf_[0] = new trajectory_tracker::Filter(
          trajectory_tracker::Filter::FILTER_LPF, filter_step_, path_.poses[0].pose.position.x);
      lpf_[1] = new trajectory_tracker::Filter(
          trajectory_tracker::Filter::FILTER_LPF, filter_step_, path_.poses[0].pose.position.y);

      for (size_t i = 0; i < path_.poses.size(); i++)
      {
        path_.poses[i].pose.position.x = lpf_[0]->in(path_.poses[i].pose.position.x);
        path_.poses[i].pose.position.y = lpf_[1]->in(path_.poses[i].pose.position.y);
      }

      delete lpf_[0];
      delete lpf_[1];
    }

    pub_path_->publish(path_);
    updateIM();
  }
  else
  {
    serial_size_ = 0;
    req_path_.filename = "";
    path_.poses.clear();
    path_.header.frame_id = "map";
  }
}

void ServerNode::spin()
{
  rclcpp::Rate loop_rate(hz_);
  trajectory_tracker_msgs::msg::TrajectoryServerStatus status;

  while (rclcpp::ok())
  {
    status.header = path_.header;
    status.filename = req_path_.filename;
    status.id = req_path_.id;
    pub_status_->publish(status);
    rclcpp::spin_some(shared_from_this());
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto serv = std::make_shared<ServerNode>();
  serv->spin();

  return 0;
}
