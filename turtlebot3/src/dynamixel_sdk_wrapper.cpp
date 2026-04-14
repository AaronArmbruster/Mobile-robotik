#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#include <iostream>

namespace turtlebot3
{

// -----------------------------------------------------------------------------
DynamixelSDKWrapper::DynamixelSDKWrapper(const char *port_name)
  : packet_handler_(dynamixel::PacketHandler::getPacketHandler()),
    port_handler_(dynamixel::PortHandler::getPortHandler(port_name))
{ }

// -----------------------------------------------------------------------------
DynamixelSDKWrapper::~DynamixelSDKWrapper()
{
  // Disable torque
  {
    uint8_t data = 0U;

    write(DXL_LEFT_ID, dxl_control_table::TORQUE_ENABLE, &data);
    write(DXL_RIGHT_ID, dxl_control_table::TORQUE_ENABLE, &data);
  }

  port_handler_->closePort();

  packet_handler_ = nullptr;
  port_handler_ = nullptr;
}

// -----------------------------------------------------------------------------
bool DynamixelSDKWrapper::init()
{
  if (!port_handler_->openPort())
    return false;

  if (!port_handler_->setBaudRate(1000000))
    return false;

  // Disable torque
  {
    uint8_t data = 0U;

    if (!write(DXL_LEFT_ID, dxl_control_table::TORQUE_ENABLE, &data))
      return false;

    if (!write(DXL_RIGHT_ID, dxl_control_table::TORQUE_ENABLE, &data))
      return false;
  }

  // Put motors in PWM mode
  {
    uint8_t data = 16U;

    if (!write(DXL_LEFT_ID, dxl_control_table::OPERATING_MODE, &data))
      return false;

    if (!write(DXL_RIGHT_ID, dxl_control_table::OPERATING_MODE, &data))
      return false;
  }

  // Enable torque
  {
    uint8_t data = 1U;

    if (!write(DXL_LEFT_ID, dxl_control_table::TORQUE_ENABLE, &data))
      return false;

    if (!write(DXL_RIGHT_ID, dxl_control_table::TORQUE_ENABLE, &data))
      return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
bool DynamixelSDKWrapper::read(
  uint8_t dxl_id,
  const ControlItem &control_item,
  uint8_t *data
)
{
  uint8_t error = 0;
  int result = packet_handler_->readTxRx(
    port_handler_,
    dxl_id,
    control_item.address_,
    control_item.length_,
    data,
    &error
  );

  if (result != COMM_SUCCESS)
  {
    std::cerr << packet_handler_->getTxRxResult(result) << std::endl;
    return false;
  }

  if (error)
  {
    std::cerr << packet_handler_->getRxPacketError(error) << std::endl;
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
bool DynamixelSDKWrapper::write(
  uint8_t dxl_id,
  const ControlItem &control_item,
  uint8_t *data
)
{
  uint8_t error = 0;
  int result = packet_handler_->writeTxRx(
    port_handler_,
    dxl_id,
    control_item.address_,
    control_item.length_,
    data,
    &error
  );

  if (result != COMM_SUCCESS)
  {
    std::cerr << packet_handler_->getTxRxResult(result) << std::endl;
    return false;
  }

  if (error)
  {
    std::cerr << packet_handler_->getRxPacketError(error) << std::endl;
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
bool DynamixelSDKWrapper::syncRead(
  const ControlItem &control_item,
  uint32_t *data_left,
  uint32_t *data_right
)
{
  dynamixel::GroupSyncRead sync_read(
    port_handler_,
    packet_handler_,
    control_item.address_,
    control_item.length_
  );

  sync_read.addParam(DXL_LEFT_ID);
  sync_read.addParam(DXL_RIGHT_ID);

  int result = sync_read.txRxPacket();

  if (result != COMM_SUCCESS)
  {
    std::cerr << packet_handler_->getTxRxResult(result) << std::endl;
    return false;
  }

  if (
    !sync_read.isAvailable(
      DXL_LEFT_ID,
      control_item.address_,
      control_item.length_
    ) ||

    !sync_read.isAvailable(
      DXL_RIGHT_ID,
      control_item.address_,
      control_item.length_
    )
  )
    return false;

  *data_left = sync_read.getData(
    DXL_LEFT_ID,
    control_item.address_,
    control_item.length_
  );

  *data_right = sync_read.getData(
    DXL_RIGHT_ID,
    control_item.address_,
    control_item.length_
  );

  return true;
}

// -----------------------------------------------------------------------------
bool DynamixelSDKWrapper::syncWrite(
  const ControlItem &control_item,
  uint8_t *data_left,
  uint8_t *data_right
)
{
  dynamixel::GroupSyncWrite sync_write(
    port_handler_,
    packet_handler_,
    control_item.address_,
    control_item.length_
  );

  sync_write.addParam(DXL_LEFT_ID, data_left);
  sync_write.addParam(DXL_RIGHT_ID, data_right);

  int result = sync_write.txPacket();

  if (result != COMM_SUCCESS)
  {
    std::cout << packet_handler_->getTxRxResult(result) << std::endl;
    return false;
  }

  return true;
}

} /* namespace turtlebot3 */