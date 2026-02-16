#ifndef _TURTLEBOT3__DYNAMIXEL_SDK_WRAPPER_HPP_
#define _TURTLEBOT3__DYNAMIXEL_SDK_WRAPPER_HPP_

#include "turtlebot3/dynamixel_control_table.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace turtlebot3
{

class DynamixelSDKWrapper
{
  // Directives ----------------------------------------------------------------
  private:
    using ControlItem = dxl_control_table::ControlItem;

  // Variables -----------------------------------------------------------------
  public:
    static constexpr uint8_t DXL_LEFT_ID = 1U;
    static constexpr uint8_t DXL_RIGHT_ID = 2U;

  private:
    dynamixel::PacketHandler *packet_handler_;
    dynamixel::PortHandler *port_handler_;

  // Methods -------------------------------------------------------------------
  public:
    DynamixelSDKWrapper(const char *port_name = "/dev/ttyACM0");
    ~DynamixelSDKWrapper();

    /**
     * @brief Initialize the motors
     * 
     * @return true If the initialization was successful.
     * @return false Otherwise
     */
    bool init();

    /**
     * @brief Reads the position encoder value of a motor 
     * 
     * @param dxl_id Motor id
     * @return The position encoder value
     */
    int32_t readPosition(uint8_t dxl_id)
    {
      int32_t data = 0;
      read(
        dxl_id,
        dxl_control_table::PRESENT_POSITION,
        reinterpret_cast<uint8_t *>(&data)
      );

      return data;
    }

    /**
     * @brief Sets the PWM value of a motor
     * 
     * @param dxl_id Motor id
     * @param pwm_value PWM value
     * @return true If the PWM value was set successfully.
     * @return false Otherwise
     */
    inline bool writePWM(uint8_t dxl_id, int16_t pwm_value)
    {
      return write(
        dxl_id,
        dxl_control_table::GOAL_PWM,
        reinterpret_cast<uint8_t *>(&pwm_value)
      );
    }

  private:
    bool read(uint8_t dxl_id, const ControlItem &control_item, uint8_t *data);
    bool write(uint8_t dxl_id, const ControlItem &control_item, uint8_t *data);
};

} /* namespace turtlebot3 */

#endif /* _TURTLEBOT3__DYNAMIXEL_SDK_WRAPPER_HPP_ */