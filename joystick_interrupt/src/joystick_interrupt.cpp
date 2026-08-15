/*
 * Copyright (c) 2015-2018, the neonavigation authors
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

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

#include "joystick_interrupt/joystick_interrupt_component_parameter.hpp"
#include "joystick_function.hpp"

namespace joystick_interrupt
{

using namespace joystick_interrupt::functions;
using Joy = sensor_msgs::msg::Joy;

class JoystickInterrupt : public rclcpp::Node
{
private:
  rclcpp::Subscription<Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_int_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::Time last_joy_received_;
  ActionDispatchTable dispatcher_;
  ButtonState state_;

  void cbJoy(const Joy::ConstSharedPtr msg)
  {
    dispatcher_.dispatch_button(msg->buttons, state_);

    if (state_.reset) {
      state_ = ButtonState(state_.mode);
    }

    if (state_.mode != Mode::JOY || state_.clutch) {
      return;
    }

    if (!state_.deadman) {
      pub_twist_->publish(Twist());
      return;
    }

    auto axes = msg->axes;
    if (state_.exclusive) {
      dispatcher_.exclude_axes(axes);
    }

    Twist cmd_vel;
    dispatcher_.dispatch_axis(axes, state_.level, cmd_vel);

    last_joy_received_ = this->now();
    pub_twist_->publish(cmd_vel);
  };

  void cbTwist(const Twist::ConstSharedPtr msg)
  {
    if (state_.mode != Mode::TWIST) {
      return;
    }

    last_joy_received_ = this->now();
    pub_twist_->publish(*msg);
  };

  void on_parameter_changed(const Params& params)
  {
    state_.refresh(params.speed_levels);
    params_ = params;
    dispatcher_ = ActionDispatchTable(params_);
  }

  void on_timeout()
  {
    if (!(state_.clutch && state_.mode == Mode::JOY)) {
      const auto dt = (now() - last_joy_received_).seconds();
      if (dt > params_.timeout) {
        pub_twist_->publish(Twist());
      }
    }
  }

public:
  JoystickInterrupt(const rclcpp::NodeOptions& options) : Node("joystick_interrupt", options)
  , last_joy_received_(now())
  {
    sub_joy_ = this->create_subscription<Joy>("joy", 1, [this](const Joy::ConstSharedPtr msg){ cbJoy(msg); });
    sub_twist_ = this->create_subscription<Twist>(
        "cmd_vel_input",
        1,[this](const Twist::ConstSharedPtr msg){ cbTwist(msg); });
    pub_twist_ = this->create_publisher<Twist>(
        "cmd_vel",
        2);
    pub_int_ = this->create_publisher<std_msgs::msg::Bool>("~/interrupt_status", 2);

    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    param_listener_->setUserCallback(
      [this](const Params& params){ this->on_parameter_changed(params); }
    );
    this->on_parameter_changed(param_listener_->get_params());

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){ on_timeout(); });
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joystick_interrupt::JoystickInterrupt)
