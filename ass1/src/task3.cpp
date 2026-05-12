#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <atomic>
#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#define RAD_ABSTAND 0.16
#define MAX_PWM 885
#define MAX_VELOCITY 0.206
#define TICKS_PER_REV 4096.0
#define RAD_DURCHMESSER 0.066
#define PI 3.1415926535

static turtlebot3::DynamixelSDKWrapper dxl;

std::atomic<double> target_v(0.0);
std::atomic<double> target_omega(0.0);
// std::atomic<int> drive_time_seconds(5);
std::atomic<bool> keep_running(true);
// std::atomic<std::chrono::steady_clock::time_point> drive_start_time;

// Funktion für den Benutzereingabe-Thread
void input_thread_fun()
{
  double v, w;
  // int drive_time;
  while (keep_running)
  {
    std::cout << "\n--- Steuerung ---" << std::endl;
    std::cout << "Eingabe: [v] [omega] (z.B. 5 0.1 0.2) oder [0 0] für Stopp." << std::endl;
    std::cout << "Zum Beenden: [-1 0]" << std::endl;
    std::cout << "> ";

    if (std::cin >> v >> w)
    {
      if (v == -1)
      {
        keep_running = false;
        target_v = 0;
        target_omega = 0;
      }
      else
      {
        target_v = v;
        target_omega = w;
        // drive_time_seconds = drive_time;
        // if (drive_time < 0)
        continue; // Ungültige Eingabe, ignoriere
        // drive_start_time = std::chrono::steady_clock::now();
      }
    }
  }
}

void drive()
{
  // Initialisierung wie in deinem Code
  double v_l, v_r;
  double u_l_tick = 0, u_r_tick = 0;
  int16_t u_l_pwm = 0, u_r_pwm = 0;
  int16_t y_l_pwm = 0, y_r_pwm = 0;
  double integral_l = 0, integral_r = 0;

  int32_t current_pos_l = 0, current_pos_r = 0;
  int32_t last_pos_l = 0, last_pos_r = 0;

  dxl.syncReadPosition(&last_pos_l, &last_pos_r);
  auto last_time = std::chrono::steady_clock::now();

  double Kp = 0.81;
  double Ki = 14.45;
  double umfang = RAD_DURCHMESSER * PI;

  std::cout << "Regler gestartet..." << std::endl;

  while (keep_running)
  {
    // if ((drive_start_time.load() + std::chrono::seconds(drive_time_seconds)) < std::chrono::steady_clock::now())
    //{
    //   target_v = 0;
    //   target_omega = 0;
    // }

    // 1. Aktuelle Zielgeschwindigkeiten aus den atomaren Variablen holen
    double v = target_v;
    double omega = target_omega;

    v_r = (v + (omega * RAD_ABSTAND / 2.0));
    v_l = (v - (omega * RAD_ABSTAND / 2.0));

    // Limitierung
    if (v_r > MAX_VELOCITY)
    {
      v_r = MAX_VELOCITY;
    }

    if (v_l > MAX_VELOCITY)
    {
      v_l = MAX_VELOCITY;
    }

    if (v_r < -MAX_VELOCITY)
    {
      v_r = -MAX_VELOCITY;
    }

    if (v_l < -MAX_VELOCITY)
    {
      v_l = -MAX_VELOCITY;
    }

    double tick_target_l = (v_l / umfang) * TICKS_PER_REV;
    double tick_target_r = (v_r / umfang) * TICKS_PER_REV;

    // 2. Sensorik & Zeit
    dxl.syncReadPosition(&current_pos_l, &current_pos_r);
    auto current_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> duration = current_time - last_time;
    double dt = duration.count();

    // 3. PI-Regler Logik
    double tick_vel_l = (current_pos_l - last_pos_l) / dt;
    double tick_vel_r = (current_pos_r - last_pos_r) / dt;

    // Fehlerberechnung
    double err_l_tick = tick_target_l - tick_vel_l;
    double err_r_tick = tick_target_r - tick_vel_r;

    // Anti-Windup & Integral
    if (std::abs(u_l_pwm) < MAX_PWM)
      integral_l += err_l_tick * dt;
    if (std::abs(u_r_pwm) < MAX_PWM)
      integral_r += err_r_tick * dt;

    // Reglerausgang in Ticks
    u_l_tick = (Kp * err_l_tick) + (Ki * integral_l);
    u_r_tick = (Kp * err_r_tick) + (Ki * integral_r);

    // Umrechnung in PWM
    u_l_pwm = (u_l_tick / TICKS_PER_REV * umfang) / MAX_VELOCITY * MAX_PWM;
    u_r_pwm = (u_r_tick / TICKS_PER_REV * umfang) / MAX_VELOCITY * MAX_PWM;

    // Output Limitierung
    y_l_pwm = std::max((int16_t)-MAX_PWM, std::min((int16_t)MAX_PWM, u_l_pwm));
    y_r_pwm = std::max((int16_t)-MAX_PWM, std::min((int16_t)MAX_PWM, u_r_pwm));

    // Motoren ansteuern
    dxl.syncWritePWM(y_l_pwm, y_r_pwm);

    // Update für nächsten Zyklus
    last_pos_l = current_pos_l;
    last_pos_r = current_pos_r;
    last_time = current_time;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Am Ende anhalten
  dxl.syncWritePWM(0, 0);
}

int main()
{
  if (!dxl.init())
  {
    std::cout << "Fehler beim Initialisieren von DXL!" << std::endl;
    return -1;
  }

  // Eingabe-Thread starten
  std::thread input_thread(input_thread_fun);

  // Regler starten
  drive();

  // Warten bis Threads fertig sind
  if (input_thread.joinable())
  {
    input_thread.join();
  }

  std::cout << "Programm beendet." << std::endl;
  return 0;
}