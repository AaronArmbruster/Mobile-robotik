#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#include <thread>
#include <iostream>

#define RAD_ABSTAND 0.16 // in meter
#define MAX_PWM 885
#define MAX_VELOCITY 0.206 // in m/s

bool drive(double v, double omega);

// -----------------------------------------------------------------------------
int main()
{
  drive(0.206, 0);

  // TODO Implement task 1
  return 0;
}

/** ************************************************************************
 * @brief
 *
 * @param v
 * @param omega
 * @return
 * @return false otherwise.
 * ************************************************************************/
bool drive(double v, double omega)
{
  turtlebot3::DynamixelSDKWrapper dxl;
  if (!dxl.init())
    return false;

  double v_r = (v + (omega * RAD_ABSTAND / 2));
  double v_l = (v - (omega * RAD_ABSTAND / 2));

  int pwm_r = v_r / MAX_VELOCITY * MAX_PWM;
  int pwm_l = v_l / MAX_VELOCITY * MAX_PWM;

  std::cout << pwm_l << " " << pwm_r;

  dxl.syncWritePWM(pwm_l, pwm_r);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  return true;
}
