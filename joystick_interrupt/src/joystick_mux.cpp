/*
 * Copyright (c) 2015-2020, the neonavigation authors
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
#include <sensor_msgs/msg/joy.hpp>

namespace joystick_interrupt
{

class JoystickMux : public rclcpp::Node
{
private:

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::TimerBase::SharedPtr timer_;
  double timeout_;
  int interrupt_button_;
  rclcpp::Time last_joy_msg_;
  bool advertised_;
  int selected_;

  void cbJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
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

  void cbTimer()
  {
    if (this->now() - last_joy_msg_ > rclcpp::Duration::from_seconds(timeout_))
    {
      selected_ = 0;
    }
  }

public:
  JoystickMux(const rclcpp::NodeOptions& options) : Node("joystick_mux", options)
  , last_joy_msg_(0L, RCL_ROS_TIME)
  {
    using std::placeholders::_1;
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, [this](const sensor_msgs::msg::Joy::SharedPtr msg){ cbJoy(msg); });

    interrupt_button_ = this->declare_parameter("interrupt_button", 5);
    timeout_ = this->declare_parameter("timeout", 0.5);
    last_joy_msg_ = this->now();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){ cbTimer(); });

    advertised_ = false;
    selected_ = 0;
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joystick_interrupt::JoystickMux)