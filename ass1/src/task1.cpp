#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#include <thread>
#include <iostream>

#define RAD_ABSTAND 0.16 // in meter
#define MAX_PWM 885
#define MAX_VELOCITY 0.206 // in m/s

static turtlebot3::DynamixelSDKWrapper dxl;

bool drive(double v, double omega)
{
  double s_r = 0.995;
  double s_l = 1;

  // Inverse Kenematik
  double v_r = (v + (omega * RAD_ABSTAND / 2)) * s_r;
  double v_l = (v - (omega * RAD_ABSTAND / 2)) * s_l;

  if (v_r > MAX_VELOCITY)
  {
    v_r = MAX_VELOCITY;
    std::cout << "v_r is too high, set to max velocity" << std::endl;
  }

  if (v_l > MAX_VELOCITY)
  {
    v_l = MAX_VELOCITY;
    std::cout << "v_l is too high, set to max velocity" << std::endl;
  }

  // Umrechnung in PWM
  int pwm_r = v_r / MAX_VELOCITY * MAX_PWM;
  int pwm_l = v_l / MAX_VELOCITY * MAX_PWM;

  std::cout << pwm_l << " " << pwm_r << std::endl;

  dxl.syncWritePWM(pwm_l, pwm_r);

  return true;
}

// -----------------------------------------------------------------------------
int main()
{
  if (!dxl.init())
    return -1;

  drive(0.1, 0.2);

  std::this_thread::sleep_for(std::chrono::seconds(5));

  return 0;
}