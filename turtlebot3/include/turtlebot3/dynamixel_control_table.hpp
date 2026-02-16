#ifndef _TURTLEBOT3__DYNAMIXEL_CONTROL_TABLE_HPP_
#define _TURTLEBOT3__DYNAMIXEL_CONTROL_TABLE_HPP_

#include <cstdint>

namespace turtlebot3
{

namespace dxl_control_table
{

struct ControlItem
{
  const uint16_t address_;
  const uint16_t length_;
};

constexpr ControlItem OPERATING_MODE = { 11U, 1U };
constexpr ControlItem TORQUE_ENABLE = { 64U, 1U };
constexpr ControlItem GOAL_PWM = { 100U, 2U };
constexpr ControlItem PRESENT_POSITION = { 132U, 4U };

} /* namespace dx_control_table */

} /* namespace turtlebot3 */

#endif /* _TURTLEBOT3__DYNAMIXEL_CONTROL_TABLE_HPP_ */