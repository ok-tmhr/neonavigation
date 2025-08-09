/*
 * Copyright (c) 2015-2020, the neonavigation authors
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
#include <sensor_msgs/msg/joy.hpp>

// #include <topic_tools/shape_shifter.h>


class JoystickMux : public rclcpp::Node
{
private:
  ros::Subscriber sub_topics_[2];
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  ros::Publisher pub_topic_;
  rclcpp::TimerBase::SharedPtr timer_;
  double timeout_;
  int interrupt_button_;
  rclcpp::Time last_joy_msg_;
  bool advertised_;
  int selected_;

  void cbJoy(const sensor_msgs::msg::Joy::Ptr msg)
  {
    if (static_cast<size_t>(interrupt_button_) >= msg->buttons.size())
    {
      RCLCPP_ERROR(this->get_logger(),
          "Out of range: number of buttons (%lu) must be greater than interrupt_button (%d).",
          msg->buttons.size(), interrupt_button_);
      return;
    }

    last_joy_msg_ = this->now();
    if (msg->buttons[interrupt_button_])
    {
      selected_ = 1;
    }
    else
    {
      selected_ = 0;
    }
  };
  void cbTopic(const std::shared_ptr<topic_tools::ShapeShifter const>& msg, int id)
  {
    if (selected_ == id)
    {
      if (!advertised_)
      {
        advertised_ = true;
        if (false)
        {
          RCLCPP_ERROR(this->get_logger(),
              "Use %s (%s%s) topic instead of %s (%s%s)",
              nh_.resolveName("mux_output", false).c_str(),
              neonavigation_common::compat::getSimplifiedNamespace(nh_).c_str(),
              "mux_output",
              pnh_.resolveName("output", false).c_str(),
              neonavigation_common::compat::getSimplifiedNamespace(pnh_).c_str(),
              "output");
          pub_topic_ = msg->advertise(pnh_, "output", 1, false);
        }
        else
        {
          pub_topic_ = msg->advertise(nh_, "mux_output", 1, false);
        }
      }
      pub_topic_->publish(*msg);
    }
  };
  void cbTimer()
  {
    if (this->now() - last_joy_msg_ > rclcpp::Duration::from_seconds(timeout_))
    {
      selected_ = 0;
    }
  }

public:
  JoystickMux()
    : Node("joystick_mux")
  {

    using std::placeholders::_1;
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&JoystickMux::cbJoy, this, _1));
    sub_topics_[0] = this->create_subscription<topic_tools::ShapeShifter>(
        "mux_input0",
        1, std::bind(&JoystickMux::cbTopic, this, _1, 0));
    sub_topics_[1] = this->create_subscription<topic_tools::ShapeShifter>(
        "mux_input1",
        1, std::bind(&JoystickMux::cbTopic, this, _1, 1));

    interrupt_button_ = this->declare_parameter("interrupt_button", 5);
    timeout_ = this->declare_parameter("timeout", 0.5);
    last_joy_msg_ = this->now();

    timer_ = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&JoystickMux::cbTimer, this));

    advertised_ = false;
    selected_ = 0;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto jy = std::make_shared<JoystickMux>();
  rclcpp::spin(jy);

  return 0;
}
