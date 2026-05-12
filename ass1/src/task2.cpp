#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#define RAD_ABSTAND 0.16 // in Meter
#define MAX_PWM 885
#define MAX_VELOCITY 0.206 // in m/s
#define TICKS_PER_REV 4096.0
#define RAD_DURCHMESSER 0.066 // im Meter
#define PI 3.1415926535

static turtlebot3::DynamixelSDKWrapper dxl;

bool drive(int drive_time_seconds, double v_l, double v_r)
{
  double u_l_tick = 0;
  double u_r_tick = 0;
  int16_t u_l_pwm = 0;
  int16_t u_r_pwm = 0;
  int16_t y_l_pwm = 0;
  int16_t y_r_pwm = 0;
  double integral_l = 0, integral_r = 0;

  int32_t current_pos_l = 0;
  int32_t current_pos_r = 0;
  auto current_time = std::chrono::steady_clock::now();

  int32_t last_pos_l = 0;
  int32_t last_pos_r = 0;
  auto last_time = std::chrono::steady_clock::now();

  std::chrono::duration<double> duration = current_time - last_time;
  double dt = duration.count();

  // 1. Zielwerte berechnen (m/s -> Ticks/s)
  double umfang = RAD_DURCHMESSER * PI;
  double tick_target_l = (v_l / umfang) * TICKS_PER_REV;
  double tick_target_r = (v_r / umfang) * TICKS_PER_REV;

  // 2. Initialisierung für den Regler

  double Kp = 0.81;
  double Ki = 14.45;

  // std::ofstream dataFile("pwm_log.csv");
  // dataFile << "PWM_Wert_links;PWM_Wert_rechts" << std::endl;

  dxl.syncReadPosition(&last_pos_l, &last_pos_r);
  last_time = std::chrono::steady_clock::now();

  // 3. Fahr-Schleife
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(drive_time_seconds))
  {
    // Positionen lesen
    dxl.syncReadPosition(&current_pos_l, &current_pos_r);
    current_time = std::chrono::steady_clock::now();

    // Zeitdifferenz
    duration = (current_time - last_time);
    dt = duration.count();

    // std::cout << dt << std::endl;

    // Aktuelle Geschwindigkeit berechnen
    double tick_vel_l = (current_pos_l - last_pos_l) / dt;
    double tick_vel_r = (current_pos_r - last_pos_r) / dt;

    // PI-Regler
    double err_l_tick = tick_target_l - tick_vel_l;
    double err_r_tick = tick_target_r - tick_vel_r;

    // Anti-Windup & Integrator
    if (!(u_l_pwm > MAX_PWM || u_l_pwm < -MAX_PWM))
      integral_l += err_l_tick * dt;

    if (!(u_r_pwm > MAX_PWM || u_r_pwm < -MAX_PWM))
      integral_r += err_r_tick * dt;

    // Regelausgang
    u_l_tick = (Kp * err_l_tick) + (Ki * integral_l);
    u_r_tick = (Kp * err_r_tick) + (Ki * integral_r);

    // Umrechnung in PWM
    u_l_pwm = (u_l_tick / TICKS_PER_REV * umfang) / MAX_VELOCITY * MAX_PWM;
    u_r_pwm = (u_r_tick / TICKS_PER_REV * umfang) / MAX_VELOCITY * MAX_PWM;

    y_l_pwm = u_l_pwm;
    y_r_pwm = u_r_pwm;

    // Limitieren
    if (u_l_pwm > MAX_PWM)
      y_l_pwm = MAX_PWM;
    else if (u_l_pwm < -MAX_PWM)
      y_l_pwm = -MAX_PWM;
    if (u_r_pwm > MAX_PWM)
      y_r_pwm = MAX_PWM;
    else if (u_r_pwm < -MAX_PWM)
      y_r_pwm = -MAX_PWM;

    // std::cout << u_l_pwm << " " << y_l_pwm << " | " << u_r_pwm << " " << y_r_pwm << " | " << err_l_tick << " : " << err_r_tick << std::endl;

    // Schreiben
    dxl.syncWritePWM(y_l_pwm, y_r_pwm);

    // Werte für nächsten Durchlauf speichern
    last_pos_l = current_pos_l;
    last_pos_r = current_pos_r;
    last_time = current_time;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Am Ende Motoren aus
  dxl.syncWritePWM(0, 0);

  return true;
}

int main()
{
  if (!dxl.init())
    return -1;

  int drive_time_seconds = 5;

  drive(drive_time_seconds, 0.15, 0.15); // max 0.206 m/s

  return 0;
}
