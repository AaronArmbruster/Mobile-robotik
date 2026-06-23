#include "tb3_util/object_detection.hpp"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

#include "turtlebot3/dynamixel_sdk_wrapper.hpp"
#include "turtlebot3/lds03_wrapper.hpp"

#define RAD_ABSTAND 0.16
#define MAX_PWM 885
#define MAX_VELOCITY 0.206
#define TICKS_PER_REV 4096.0
#define RAD_DURCHMESSER 0.066
#define PI 3.1415926535

constexpr double STOP_DISTANCE = 0.25;
constexpr double SAFETY_DISTANCE = 0.20; // 5cm Sicherheitsabstand zu Hindernissen
constexpr double ANGLE_THRESHOLD = 0.35; // Ausrichtungstoleranz

static turtlebot3::DynamixelSDKWrapper dxl;
std::mutex dxl_mutex;

turtlebot3::LDS03Packet latest_scan;
std::atomic<bool> driving(true);
std::atomic<double> target_v(0.0);
std::atomic<double> target_omega(0.0);

enum class State
{
  SEARCHING,
  DRIVING_TO_GOAL,
  OBJECT_COLLISION,
  STOPPED
};

// Regler Funktion aus Assignment 2
void regler()
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

  while (driving)
  {
    double v = target_v;
    double omega = target_omega;
    v_r = (v + (omega * RAD_ABSTAND / 2.0));
    v_l = (v - (omega * RAD_ABSTAND / 2.0));

    if (v_r > MAX_VELOCITY)
      v_r = MAX_VELOCITY;
    if (v_l > MAX_VELOCITY)
      v_l = MAX_VELOCITY;
    if (v_r < -MAX_VELOCITY)
      v_r = -MAX_VELOCITY;
    if (v_l < -MAX_VELOCITY)
      v_l = -MAX_VELOCITY;

    double tick_target_l = (v_l / umfang) * TICKS_PER_REV;
    double tick_target_r = (v_r / umfang) * TICKS_PER_REV;

    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      dxl.syncReadPosition(&current_pos_l, &current_pos_r);
    }
    auto current_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> duration = current_time - last_time;
    double dt = duration.count();

    if (dt <= 0.0)
      dt = 0.01;

    double tick_vel_l = (current_pos_l - last_pos_l) / dt;
    double tick_vel_r = (current_pos_r - last_pos_r) / dt;

    double err_l_tick = tick_target_l - tick_vel_l;
    double err_r_tick = tick_target_r - tick_vel_r;

    if (std::abs(u_l_pwm) < MAX_PWM)
      integral_l += err_l_tick * dt;
    if (std::abs(u_r_pwm) < MAX_PWM)
      integral_r += err_r_tick * dt;

    u_l_tick = (Kp * err_l_tick) + (Ki * integral_l);
    u_r_tick = (Kp * err_r_tick) + (Ki * integral_r);

    u_l_pwm = (u_l_tick / TICKS_PER_REV * umfang) / MAX_VELOCITY * MAX_PWM;
    u_r_pwm = (u_r_tick / TICKS_PER_REV * umfang) / MAX_VELOCITY * MAX_PWM;

    y_l_pwm = std::max((int16_t)-MAX_PWM, std::min((int16_t)MAX_PWM, u_l_pwm));
    y_r_pwm = std::max((int16_t)-MAX_PWM, std::min((int16_t)MAX_PWM, u_r_pwm));

    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      dxl.syncWritePWM(y_l_pwm, y_r_pwm);
    }

    last_pos_l = current_pos_l;
    last_pos_r = current_pos_r;
    last_time = current_time;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  {
    std::lock_guard<std::mutex> lock(dxl_mutex);
    dxl.syncWritePWM(0, 0);
  }
}

void callback(const turtlebot3::LDS03Packet &packet)
{
  std::lock_guard<std::mutex> lock(dxl_mutex);
  latest_scan = packet;
}

// FUnktion prüft ob ein Hindernis im Weg steht.
bool check_for_obstacles(const turtlebot3::LDS03Packet &scan, const tb3_util::Objects &objects, const tb3_util::Object &goal)
{
  for (const auto &obj : objects)
  {
    if (obj == goal)
      continue;

    if (!obj.empty())
    {
      size_t mid = obj.size() / 2;
      double dist = scan.data[obj[mid]].distance;

      // Wir berechnen den Wikel des Objekts, relativ zum Roboter.
      double angle = scan.header.angle_start + (obj[mid] * scan.header.angle_increment);
      angle = std::fmod(angle, 2.0 * M_PI);
      if (angle > M_PI)
        angle -= 2.0 * M_PI;

      if (dist < SAFETY_DISTANCE && std::abs(angle) < (M_PI / 2.0))
      {
        return true;
      }
    }
  }
  return false;
}

// Funktion, in der der Roboter das Zeil sucht
void search()
{
  std::cout << "Suche nach Ziel..." << std::endl;
  target_v = 0.1;
  target_omega = 0.15;
}

void avoid_obstacle(const turtlebot3::LDS03Packet &scan, const tb3_util::Objects &objects, const tb3_util::Object &goal)
{
  std::cout << "Hindernis erkannt, Roboter weicht aus..." << std::endl;

  double angle_to_goal = 0.0;
  bool goal_visible = !goal.empty();

  if (goal_visible)
  {
    size_t goal_mid = goal.size() / 2;
    angle_to_goal = scan.header.angle_start + (goal[goal_mid] * scan.header.angle_increment);
    angle_to_goal = std::fmod(angle_to_goal, 2.0 * M_PI);
    if (angle_to_goal > M_PI)
      angle_to_goal -= 2.0 * M_PI;
    angle_to_goal = -angle_to_goal;
  }

  // wir schauen welches Hindernis am nächsten ist
  double min_dist_obs = 999.0;
  size_t dangerous_lds_index = 0;
  bool obstacle_found = false;

  for (const auto &obj : objects)
  {
    if (obj == goal || obj.empty())
      continue;

    for (size_t index : obj)
    {
      double d = scan.data[index].distance;
      if (d > 0.05 && d < min_dist_obs)
      {
        min_dist_obs = d;
        dangerous_lds_index = index;
        obstacle_found = true;
      }
    }
  }

  // Wir berechnen einen Abstoßungsvektor, der vom Hindernis zum Roboter zeigt.
  double avoidance_angle_offset = 0.0;
  if (obstacle_found)
  {
    double angle_to_obs = scan.header.angle_start + (dangerous_lds_index * scan.header.angle_increment);
    angle_to_obs = std::fmod(angle_to_obs, 2.0 * M_PI);
    if (angle_to_obs > M_PI)
      angle_to_obs -= 2.0 * M_PI;
    angle_to_obs = -angle_to_obs;

    // Sklarierung des Abstoßungsvektors
    double repulsion_weight = 0.8 / (min_dist_obs + 0.01);

    if (angle_to_obs >= 0)
    {
      avoidance_angle_offset = -0.5 * repulsion_weight;
    }
    else
    {
      avoidance_angle_offset = 0.5 * repulsion_weight;
    }
  }

  double final_target_angle = angle_to_goal + avoidance_angle_offset;

  // Sicherheitscheck, an der Wand
  if (min_dist_obs < (SAFETY_DISTANCE * 0.7))
  {
    target_v = -0.05;
    target_omega = 0.4 * avoidance_angle_offset;
  }
  else
  {
    target_v = 0.06;
    target_omega = 0.4 * final_target_angle;
  }
}

// diese funktion dient zur Zielansteuerung.
void drive_to_goal(const turtlebot3::LDS03Packet &scan, const tb3_util::Object &goal)
{
  size_t middle_index = goal.size() / 2;
  size_t lds_index = goal[middle_index];

  double angle_to_goal = scan.header.angle_start + (lds_index * scan.header.angle_increment);

  angle_to_goal = std::fmod(angle_to_goal, 2.0 * M_PI);

  if (angle_to_goal > M_PI)
    angle_to_goal -= 2.0 * M_PI;

  angle_to_goal = -angle_to_goal;

  double distance_to_goal = scan.data[lds_index].distance;

  if (distance_to_goal <= STOP_DISTANCE)
  {
    target_v = 0.0;
    target_omega = 0.0;
    return;
  }

  // Lenkbefehle werden gesetzt
  if (std::abs(angle_to_goal) > ANGLE_THRESHOLD)
  {
    if (distance_to_goal < 0.5)
    {
      target_v = 0.15 * distance_to_goal;
    }
    else
    {
      // target_v = 0.0;
      target_v = 0.05 * distance_to_goal;
    }
    target_omega = 0.3 * angle_to_goal;
  }
  else
  {
    target_v = 0.15 * distance_to_goal;
    target_omega = 0;
  }
}

// Unsere State Machine
void state_machine()
{
  State current_state = State::SEARCHING;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  turtlebot3::LDS03Packet scan;

  while (driving)
  {
    {
      std::lock_guard<std::mutex> lock(dxl_mutex);
      scan = latest_scan;
    }

    tb3_util::Objects objects = tb3_util::detectObjects(scan);
    tb3_util::Object goal = tb3_util::getGoal(scan, objects);

    bool obstacle_detected = check_for_obstacles(scan, objects, goal);
    bool goal_detected = !goal.empty();

    double distance_to_goal = 999.0;
    if (goal_detected)
    {
      distance_to_goal = scan.data[goal[goal.size() / 2]].distance;
    }

    if (obstacle_detected && current_state != State::STOPPED)
    {
      current_state = State::OBJECT_COLLISION;
      std::cout << "State: Hindernis erkannt" << std::endl;
    }
    else if (goal_detected && distance_to_goal <= STOP_DISTANCE)
    {
      current_state = State::STOPPED;
      std::cout << "State: Ziel erreicht" << std::endl;
    }
    else if (current_state == State::OBJECT_COLLISION)
    {
      current_state = State::SEARCHING;
      std::cout << "State: Suche Ziel" << std::endl;
    }
    else if (current_state == State::SEARCHING && goal_detected)
    {
      current_state = State::DRIVING_TO_GOAL;
      std::cout << "State: Fahre zum Ziel" << std::endl;
    }
    else if (current_state == State::DRIVING_TO_GOAL && !goal_detected)
    {
      current_state = State::SEARCHING;
      std::cout << "State: Suche Ziel" << std::endl;
    }

    switch (current_state)
    {
    case State::SEARCHING:
      search();
      break;

    case State::DRIVING_TO_GOAL:
      drive_to_goal(scan, goal);
      break;

    case State::OBJECT_COLLISION:
      avoid_obstacle(scan, objects, goal);
      break;

    case State::STOPPED:
      std::cout << "Roboter stoppt endgültig." << std::endl;
      target_v = 0.0;
      target_omega = 0.0;
      driving = false;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

// -----------------------------------------------------------------------------
int main()
{
  turtlebot3::LDS03Wrapper lds(std::bind(&callback, std::placeholders::_1));

  if (!lds.init())
    return -1;

  if (!dxl.init())
  {
    std::cout << "Fehler beim Initialisieren von DXL!" << std::endl;
    return -1;
  }

  std::thread regler_thread(regler);

  state_machine();

  if (regler_thread.joinable())
    regler_thread.join();

  return 0;
}