#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#include <thread>

// -----------------------------------------------------------------------------
int main()
{
  turtlebot3::DynamixelSDKWrapper dxl;
  if(!dxl.init())
    return -1;

  dxl.syncWritePWM(300,300);
  dxl.readPosition(dxl.DXL_LEFT_ID);

  std::this_thread::sleep_for(std::chrono::seconds(5));

  dxl.syncWritePWM(-300,300);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  // TODO Implement task 1
  return 0;
}
