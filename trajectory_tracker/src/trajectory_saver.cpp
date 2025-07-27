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
#include <nav_msgs/msg/path.hpp>

class SaverNode : public rclcpp::Node
{
public:
  SaverNode();
  ~SaverNode();
  void save();

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

  std::string topic_path_;
  std::string filename_;
  bool saved_;
  void cbPath(const nav_msgs::msg::Path::ConstPtr& msg);
};

SaverNode::SaverNode()
  : rclcpp::Node("trajectory_saver")
  , saved_(false)
{
  topic_path_ = this->declare_parameter<std::string>("path", "recpath");
  filename_ = this->declare_parameter<std::string>("file", "a.path");

  using std::placeholders::_1;
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", 10, std::bind(&SaverNode::cbPath, this, _1));
}
SaverNode::~SaverNode()
{
}

void SaverNode::cbPath(const nav_msgs::msg::Path::ConstPtr& msg)
{
  if (saved_)
    return;
  std::ofstream ofs(filename_.c_str());

  if (!ofs)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open %s", filename_.c_str());
    return;
  }

  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<nav_msgs::msg::Path> serializer;
  serializer.serialize_message(msg.get(), serialized_msg.get());

  RCLCPP_INFO(this->get_logger(), "Size: %d\n", (int)serialized_msg->size());

  ofs.write(reinterpret_cast<char*>(serialized_msg->get_rcl_serialized_message().buffer), serialized_msg->size());

  saved_ = true;
}

void SaverNode::save()
{
  rclcpp::Rate loop_rate(5);
  RCLCPP_INFO(this->get_logger(), "Waiting for the path");

  while (rclcpp::ok())
  {
    rclcpp::spin_some(shared_from_this());
    loop_rate.sleep();
    if (saved_)
      break;
  }
  RCLCPP_INFO(this->get_logger(), "Path saved");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto rec = std::make_shared<SaverNode>();
  rec->save();

  return 0;
}
