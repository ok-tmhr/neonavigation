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

#include "joystick_interrupt/joystick_mux_component_parameter.hpp"

namespace joystick_interrupt
{

class JoystickMux : public rclcpp::Node
{
private:

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  std::array<std::shared_ptr<rclcpp::GenericSubscription>, 2> sub_topics_;
  std::shared_ptr<rclcpp::GenericPublisher> pub_topic_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<joystick_mux::ParamListener> param_listener_;
  joystick_mux::Params params_;
  rclcpp::Time last_joy_received_;
  int selected_;

  void cbJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    if (static_cast<size_t>(params_.button) >= msg->buttons.size())
    {
      RCLCPP_ERROR(this->get_logger(),
          "Parameter 'button' (%ld) exceeds available button count (%lu).",
          params_.button, msg->buttons.size());
      return;
    }

    last_joy_received_ = this->now();
    selected_ = msg->buttons[params_.button] ? 1 : 0;
  };

  void cbTopic(const std::shared_ptr<rclcpp::SerializedMessage> msg, const int id)
  {
    if (selected_ == id)
    {
      pub_topic_->publish(*msg);
    }
  }

  void cbTimer()
  {
    const auto dt = (this->now() - last_joy_received_).seconds();
    if (dt > params_.timeout)
    {
      selected_ = 0;
    }
  }

public:
  JoystickMux(const rclcpp::NodeOptions& options) : Node("joystick_mux", options)
  , last_joy_received_(0L, RCL_ROS_TIME)
  , selected_(0)
  {
    param_listener_ = std::make_shared<joystick_mux::ParamListener>(this->get_node_parameters_interface());
    param_listener_->setUserCallback([this](const joystick_mux::Params& p){ params_ = p; });
    params_ = param_listener_->get_params();

    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, [this](const sensor_msgs::msg::Joy::SharedPtr msg){ cbJoy(msg); });

    const auto& input = params_.mux_input;
    for (size_t i = 0; i < input.topics.size(); i++) {
      const int id = static_cast<int>(i);
      sub_topics_[i] = this->create_generic_subscription(
          input.topics[i], input.type, 1,
          [this, id](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            cbTopic(msg, id);
          });
    }
    pub_topic_ = this->create_generic_publisher("mux_output", input.type, 1);

    last_joy_received_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){ cbTimer(); });
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joystick_interrupt::JoystickMux)