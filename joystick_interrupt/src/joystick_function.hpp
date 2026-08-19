#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "joystick_interrupt/joystick_interrupt_component_parameter.hpp"

namespace joystick_interrupt::functions
{

enum class Mode {
  JOY,
  TWIST,
};

struct ButtonState {
  int max_level;
  int level;
  int last_mode_button;
  bool deadman;
  bool clutch;
  bool exclusive;
  bool reset;
  Mode mode;

  ButtonState(Mode mode_ = Mode::JOY)
      : max_level(0), level(0), last_mode_button(0), deadman(false),
        clutch(false), exclusive(false), reset(false), mode(mode_)
  {
  }

  void refresh(const int levels)
  {
    deadman = false;
    clutch = false;
    exclusive = false;
    reset = false;
    max_level = levels / 2;
    level = std::clamp(level, -max_level, max_level);
  }
};

constexpr size_t AXIS_COUNT = 16UL;
constexpr size_t BUTTON_COUNT = 16UL;
using Twist = geometry_msgs::msg::Twist;
using Velocity = Params::Axis::MapAxisBindings;
using AxisFunction = void (*)(const double, const Velocity&, const int, Twist&);
using ButtonFunction = void (*)(const int, ButtonState&);
using JoyButtons = std::vector<int>;
using JoyAxes = std::vector<float>;

namespace axis
{

[[nodiscard]]
inline double get_velocity(const double value, const Velocity& v,
                           const int level)
{
  return value * v.max * (1 + v.increment * level);
}

void set_linear_x(const double value, const Velocity& v, const int level,
                  Twist& msg)
{
  msg.linear.x = get_velocity(value, v, level);
}

void set_linear_y(const double value, const Velocity& v, const int level,
                  Twist& msg)
{
  msg.linear.y = get_velocity(value, v, level);
}

void set_angular_z(const double value, const Velocity& v, const int level,
                   Twist& msg)
{
  msg.angular.z = get_velocity(value, v, level);
}

void noop(const double, const Velocity&, const int, Twist&) {}

} // namespace axis

namespace button
{
void speed_up(const int value, ButtonState& st)
{
  if (value && st.level < st.max_level) {
    st.level++;
  }
}

void speed_down(const int value, ButtonState& st)
{
  if (value && st.level > -st.max_level) {
    st.level--;
  }
}

void deadman(const int value, ButtonState& st) { st.deadman = value; }

void switch_mode(const int value, ButtonState& st)
{
  if (!st.last_mode_button && value) {
    switch (st.mode) {
    case Mode::JOY:
      st.mode = Mode::TWIST;
      break;
    case Mode::TWIST:
      st.mode = Mode::JOY;
      break;
    }
  }
  st.last_mode_button = value;
}

void clutch(const int value, ButtonState& st) { st.clutch = value; }

void exclude(const int value, ButtonState& st) { st.exclusive = value; }

void reset(const int value, ButtonState& st) { st.reset = value; }

void noop(const int, ButtonState&) {}

} // namespace button

static constexpr std::array<std::pair<std::string_view, AxisFunction>, 3>
    axis_map{{
        {"linear_velocity_x", axis::set_linear_x},
        {"linear_velocity_y", axis::set_linear_y},
        {"angular_velocity", axis::set_angular_z},
    }};

AxisFunction find_axis_function(std::string_view name)
{
  for (const auto& [key, fn] : axis_map) {
    if (key == name) {
      return fn;
    }
  }
  return axis::noop;
}

static constexpr std::array<std::pair<std::string_view, ButtonFunction>, 7>
    button_map{{
        {"speed_up", button::speed_up},
        {"speed_down", button::speed_down},
        {"deadman", button::deadman},
        {"mode", button::switch_mode},
        {"clutch", button::clutch},
        {"exclude", button::exclude},
        {"reset", button::reset},
    }};

ButtonFunction find_button_function(std::string_view name)
{
  for (const auto& [key, fn] : button_map) {
    if (key == name) {
      return fn;
    }
  }
  return button::noop;
}

struct ActionDispatchTable {
  std::array<AxisFunction, AXIS_COUNT> axis_actions;
  std::array<ButtonFunction, BUTTON_COUNT> button_actions;
  std::array<Velocity, AXIS_COUNT> axis_velocity;

  ActionDispatchTable()
  {
    axis_actions.fill(axis::noop);
    button_actions.fill(button::noop);
  }

  ActionDispatchTable(const Params& params)
  {
    axis_actions.fill(axis::noop);
    button_actions.fill(button::noop);

    for (size_t i = 0; i < params.axis_bindings.size(); i++) {
      const auto name = params.axis_bindings[i];
      const auto v = params.axis.axis_bindings_map.at(name);
      axis_velocity[i] = v;
      if (v.enable) {
        axis_actions[i] = find_axis_function(name);
      }
    }

    for (size_t i = 0; i < params.button_bindings.size(); i++) {
      const auto name = params.button_bindings[i];
      const auto enable = params.button.button_bindings_map.at(name).enable;
      if (enable) {
        button_actions[i] = find_button_function(name);
      }
    }
  }

  void dispatch_button(const JoyButtons& buttons, ButtonState& state) const
  {
    for (size_t i = 0; i < buttons.size(); i++) {
      button_actions[i](buttons[i], state);
    }
  }

  void dispatch_axis(const JoyAxes& axes, const int level, Twist& msg) const
  {
    for (size_t i = 0; i < axes.size(); i++) {
      axis_actions[i](axes[i], axis_velocity[i], level, msg);
    }
  }

  void exclude_axes(JoyAxes& axes) const
  {
    std::vector<size_t> indices;
    indices.reserve(3);
    for (size_t i = 0; i < axes.size(); i++) {
      if (axis_velocity[i].exclusive) {
        indices.push_back(i);
      }
    }
    if (!indices.empty()) {
      auto it = std::max_element(indices.begin(), indices.end(),
                                 [&](size_t a, size_t b) {
                                   return std::abs(axes[a]) < std::abs(axes[b]);
                                 });
      const auto max_i = *it;
      std::for_each(indices.begin(), indices.end(), [&](size_t i) {
        if (i != max_i) {
          axes[i] = 0.0f;
        }
      });
    }
  }
};

} // namespace joystick_interrupt::functions