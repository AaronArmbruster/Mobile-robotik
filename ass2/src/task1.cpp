#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <mutex>
#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#define RAD_ABSTAND 0.16
#define MAX_PWM 885
#define MAX_VELOCITY 0.206
#define TICKS_PER_REV 4096.0
#define RAD_DURCHMESSER 0.066
#define PI 3.1415926535

static turtlebot3::DynamixelSDKWrapper dxl;
std::mutex dxl_mutex; // Mutex für sicheren Zugriff auf DXL in mehreren Threads

std::atomic<double> target_v(0.0);
std::atomic<double> target_omega(0.0);
std::atomic<bool> keep_running(true);
std::atomic<bool> odometry_running(false);

void drive_with_time(double v, double omega, int drive_time_seconds);
void drive_wasd();

// Setzt das Terminal in den "Raw-Modus" um eingabe sofort zulesen
void set_terminal_raw(bool enable)
{
  static struct termios oldt, newt;
  if (enable)
  {
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Schaltet Zeilenpufferung und lokales Echo aus
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  }
  else
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
}

// Prüft, ob eine Taste im Puffer liegt
bool kbhit()
{
  struct timeval tv = {0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

// Logik für die flüssige WASD-Steuerung via Tasten-Nachleuchten
void handle_wasd_mode()
{
  std::cout << "\n=== WASD MODUS AKTIVIERT ===" << std::endl;
  std::cout << "W = Vorwärts, S = Rückwärts, A = Links, D = Rechts" << std::endl;
  std::cout << "Tasten gedrückt halten für Kurvenfahrten." << std::endl;
  std::cout << "Taste [Q] drücken, um den WASD-Modus zu VERLASSEN." << std::endl;

  set_terminal_raw(true);

  const double CONST_V = 0.1;     // 0.1 m/s
  const double CONST_OMEGA = 0.4; // Rad/s für die Drehung

  // "Aktivitäts-Zähler" für jede Richtung
  int w_active = 0;
  int s_active = 0;
  int a_active = 0;
  int d_active = 0;

  // Überbrückt die Linux-Tastatur-Wiederholrate (25 * 10ms = 250ms Fenster)
  const int ACTIVE_TIMEOUT = 50;

  while (keep_running)
  {
    // Alle im Puffer liegenden Zeichen sofort verarbeiten
    while (kbhit())
    {
      char c = getchar();

      if (c == 'q' || c == 'Q')
      {
        goto exit_wasd; // Sauberer Ausbruch aus den verschachtelten Schleifen
      }

      // Wenn eine Taste gelesen wird, aktivieren wir das Zeitfenster dafür
      if (c == 'w' || c == 'W')
      {
        w_active = ACTIVE_TIMEOUT;
        s_active = 0;
      }
      if (c == 's' || c == 'S')
      {
        s_active = ACTIVE_TIMEOUT;
        w_active = 0;
      }
      if (c == 'a' || c == 'A')
      {
        a_active = ACTIVE_TIMEOUT;
        d_active = 0;
      }
      if (c == 'd' || c == 'D')
      {
        d_active = ACTIVE_TIMEOUT;
        a_active = 0;
      }
    }

    // Geschwindigkeiten basierend auf den aktiven Zeitfenstern berechnen
    double v = 0.0;
    double omega = 0.0;

    if (w_active > 0)
      v += CONST_V;
    if (s_active > 0)
      v -= CONST_V;
    if (a_active > 0)
      omega += CONST_OMEGA;
    if (d_active > 0)
      omega -= CONST_OMEGA;

    // Werte an den Regler-Thread übergeben
    target_v = v;
    target_omega = omega;

    // Zähler für die nächste Runde herabsetzen
    if (w_active > 0)
      w_active--;
    if (s_active > 0)
      s_active--;
    if (a_active > 0)
      a_active--;
    if (d_active > 0)
      d_active--;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

exit_wasd:
  // Zurück zum normalen Terminal-Modus
  set_terminal_raw(false);
  target_v = 0.0;
  target_omega = 0.0;
  odometry_running = false;
  std::cout << "\n=== WASD MODUS BEENDET ===\n"
            << std::endl;
}

void odometry()
{
  double v_l, v_r;
  int32_t current_pos_l = 0, current_pos_r = 0;
  int32_t last_pos_l = 0, last_pos_r = 0;
  double delta_theta = 0;
  double delta_s = 0;
  double current_theta = 0, last_theta = 0;
  double current_x = 0, current_y = 0;
  double last_x = 0, last_y = 0;
  double delta_s_l = 0, delta_s_r = 0;
  double tick_diff_l = 0, tick_diff_r = 0;

  double umfang = RAD_DURCHMESSER * PI;

  {
    std::lock_guard<std::mutex> lock(dxl_mutex);
    dxl.syncReadPosition(&last_pos_l, &last_pos_r);
  }

  while (odometry_running)
  {
    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      dxl.syncReadPosition(&current_pos_l, &current_pos_r);
    }

    tick_diff_l = (current_pos_l - last_pos_l);
    tick_diff_r = (current_pos_r - last_pos_r);

    delta_s_l = tick_diff_l / TICKS_PER_REV * umfang;
    delta_s_r = tick_diff_r / TICKS_PER_REV * umfang;

    delta_s = (delta_s_l + delta_s_r) / 2.0;
    delta_theta = (delta_s_r - delta_s_l) / RAD_ABSTAND;

    current_x = last_x - delta_s * sin(last_theta + delta_theta / 2.0);
    current_y = last_y + delta_s * cos(last_theta + delta_theta / 2.0);

    current_theta = last_theta + delta_theta;

    // std::cout << "Odometry - X: " << current_x << " m, Y: " << current_y << " m, Theta: " << current_theta << " rad" << std::endl;

    last_pos_l = current_pos_l;
    last_pos_r = current_pos_r;
    last_x = current_x;
    last_y = current_y;
    last_theta = current_theta;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // Koordinate der vorderen Kante
  current_x = current_x - 0.031 * sin(current_theta);
  current_y = current_y + 0.031 * cos(current_theta);
  std::cout << "Odometry - X: " << current_x << " m, Y: " << current_y << " m, Theta: " << current_theta << " rad" << std::endl;
}

// Funktion für den Benutzereingabe-Thread
void input_thread_fun()
{
  std::string input;
  while (keep_running)
  {
    std::cout << "\n--- Hauptmenü Steuerung ---" << std::endl;
    std::cout << "Optionen:" << std::endl;
    std::cout << "  [1] Manuelle Werte eingeben (v omega Zeit)" << std::endl;
    std::cout << "  [2] WASD-Modus starten (Flüssige Fahrt, Konstant 0.1 m/s)" << std::endl;
    std::cout << "  [-1] Programm beenden" << std::endl;
    std::cout << "> ";

    std::cin >> input;

    if (input == "-1")
    {
      keep_running = false;
      target_v = 0;
      target_omega = 0;
    }
    else if (input == "1")
    {
      double v, w;
      int time;
      std::cout << "Eingabe: [v] [omega] [zeit] (z.B. 0.1 0.2 5):\n> ";
      if (std::cin >> v >> w >> time)
      {
        odometry_running = true;
        std::thread odometry_thread(odometry);

        drive_with_time(v, w, time);
        odometry_running = false;

        if (odometry_thread.joinable())
        {
          odometry_thread.join();
        }
      }
    }
    else if (input == "2")
    {
      odometry_running = true;
      std::thread odometry_thread(odometry);
      std::thread drive_thread(drive_wasd);
      handle_wasd_mode();

      if (odometry_thread.joinable())
      {
        odometry_thread.join();
      }
      if (drive_thread.joinable())
      {
        drive_thread.join();
      }
    }
  }
}

void drive_with_time(double v, double omega, int drive_time_seconds)
{
  double v_l, v_r;
  double u_l_tick = 0, u_r_tick = 0;
  int16_t u_l_pwm = 0, u_r_pwm = 0;
  int16_t y_l_pwm = 0, y_r_pwm = 0;
  double integral_l = 0, integral_r = 0;

  int32_t current_pos_l = 0, current_pos_r = 0;
  int32_t last_pos_l = 0, last_pos_r = 0;

  {
    std::lock_guard<std::mutex> lock(dxl_mutex);
    dxl.syncReadPosition(&last_pos_l, &last_pos_r);
  }

  auto last_time = std::chrono::steady_clock::now();

  double Kp = 0.81;
  double Ki = 14.45;
  double umfang = RAD_DURCHMESSER * PI;

  std::cout << "Regler gestartet..." << std::endl;
  auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() < (start_time + std::chrono::seconds(drive_time_seconds)))
  {
    // Inverse Kinematik
    v_r = (v + (omega * RAD_ABSTAND / 2.0));
    v_l = (v - (omega * RAD_ABSTAND / 2.0));

    // Limitierung auf maximale Motorgeschwindigkeit
    if (v_r > MAX_VELOCITY)
      v_r = MAX_VELOCITY;
    if (v_l > MAX_VELOCITY)
      v_l = MAX_VELOCITY;
    if (v_r < -MAX_VELOCITY)
      v_r = -MAX_VELOCITY;
    if (v_l < -MAX_VELOCITY)
      v_l = -MAX_VELOCITY;

    // Zielwerte in Ticks/s umrechnen
    double tick_target_l = (v_l / umfang) * TICKS_PER_REV;
    double tick_target_r = (v_r / umfang) * TICKS_PER_REV;

    // Positionen & Zeit lesen
    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      dxl.syncReadPosition(&current_pos_l, &current_pos_r);
    }
    auto current_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> duration = current_time - last_time;
    double dt = duration.count();

    // Falls dt extrem klein ist (Verhindert Division durch Null)
    if (dt <= 0.0)
      dt = 0.01;

    // Aktuelle Geschwindigkeit in Ticks/s berechnen
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
    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      dxl.syncWritePWM(y_l_pwm, y_r_pwm);
    }

    // Update für nächsten Zyklus
    last_pos_l = current_pos_l;
    last_pos_r = current_pos_r;
    last_time = current_time;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Beim Verlassen des Programms Motoren sicher stoppen
  {
    std::lock_guard<std::mutex> lock(dxl_mutex);
    dxl.syncWritePWM(0, 0);
  }
}

void drive_wasd()
{
  double v_l, v_r;
  double u_l_tick = 0, u_r_tick = 0;
  int16_t u_l_pwm = 0, u_r_pwm = 0;
  int16_t y_l_pwm = 0, y_r_pwm = 0;
  double integral_l = 0, integral_r = 0;

  int32_t current_pos_l = 0, current_pos_r = 0;
  int32_t last_pos_l = 0, last_pos_r = 0;

  {
    std::lock_guard<std::mutex> lock(dxl_mutex);
    dxl.syncReadPosition(&last_pos_l, &last_pos_r);
  }

  auto last_time = std::chrono::steady_clock::now();

  double Kp = 0.81;
  double Ki = 14.45;
  double umfang = RAD_DURCHMESSER * PI;

  std::cout << "Regler gestartet..." << std::endl;

  while (keep_running && odometry_running)
  {
    double v = target_v;
    double omega = target_omega;

    // Inverse Kinematik
    v_r = (v + (omega * RAD_ABSTAND / 2.0));
    v_l = (v - (omega * RAD_ABSTAND / 2.0));

    // Limitierung auf maximale Motorgeschwindigkeit
    if (v_r > MAX_VELOCITY)
      v_r = MAX_VELOCITY;
    if (v_l > MAX_VELOCITY)
      v_l = MAX_VELOCITY;
    if (v_r < -MAX_VELOCITY)
      v_r = -MAX_VELOCITY;
    if (v_l < -MAX_VELOCITY)
      v_l = -MAX_VELOCITY;

    // Zielwerte in Ticks/s umrechnen
    double tick_target_l = (v_l / umfang) * TICKS_PER_REV;
    double tick_target_r = (v_r / umfang) * TICKS_PER_REV;

    // Positionen & Zeit lesen
    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      dxl.syncReadPosition(&current_pos_l, &current_pos_r);
    }
    auto current_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> duration = current_time - last_time;
    double dt = duration.count();

    // Falls dt extrem klein ist (Verhindert Division durch Null)
    if (dt <= 0.0)
      dt = 0.01;

    // Aktuelle Geschwindigkeit in Ticks/s berechnen
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
    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      dxl.syncWritePWM(y_l_pwm, y_r_pwm);
    }

    // Update für nächsten Zyklus
    last_pos_l = current_pos_l;
    last_pos_r = current_pos_r;
    last_time = current_time;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Beim Verlassen des Programms Motoren sicher stoppen
  {
    std::lock_guard<std::mutex> lock(dxl_mutex);
    dxl.syncWritePWM(0, 0);
  }
}

int main()
{
  if (!dxl.init())
  {
    std::cout << "Fehler beim Initialisieren von DXL!" << std::endl;
    return -1;
  }

  input_thread_fun();

  std::cout << "Programm beendet." << std::endl;
  return 0;
}
