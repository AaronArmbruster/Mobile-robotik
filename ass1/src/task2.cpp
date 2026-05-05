#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "turtlebot3/dynamixel_sdk_wrapper.hpp"

#define RAD_ABSTAND 0.16 // in meter
#define MAX_PWM 885
#define MAX_VELOCITY 0.206 // in m/s
#define TICKS_PER_REV 4096.0
#define RAD_DURCHMESSER 0.066
#define PI 3.1415926535

bool drive(double v_l, double v_r) {
    turtlebot3::DynamixelSDKWrapper dxl;
    
    if (!dxl.init())
      return false;


    int drive_time_seconds = 5;
    int iteration_time_ms = 10;

      // 1. Zielwerte berechnen (m/s -> Ticks/s)
    double umfang = RAD_DURCHMESSER * PI;
    double target_l = (v_l / umfang) * TICKS_PER_REV;
    double target_r = (v_r / umfang) * TICKS_PER_REV;

    // 2. Initialisierung für den Regler

    double Kp = 0.8;
    double Ki = 14.28;
    double integral_l = 0, integral_r = 0;
    double dt = iteration_time_ms / 1000.0; 

    std::ofstream dataFile("pwm_log.csv");
    dataFile << "PWM_Wert_links;PWM_Wert_rechts" << std::endl;

    int32_t last_pos_l = dxl.readPosition(dxl.DXL_LEFT_ID);
    int32_t last_pos_r = dxl.readPosition(dxl.DXL_RIGHT_ID);

    int32_t pos_l =0;
    int32_t pos_r =0;

    // 3. Fahr-Schleife
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(drive_time_seconds)) {
        // Positionen lesen
        dxl.syncReadPosition(&pos_l, &pos_r);

        // Aktuelle Geschwindigkeit berechnen
        double vel_l = (pos_l - last_pos_l) / dt;
        double vel_r = (pos_r - last_pos_r) / dt;

        // PI-Regler
        double err_l = target_l - vel_l;
        double err_r = target_r - vel_r;

        double err_l_pwm = (err_l / TICKS_PER_REV * umfang)/MAX_VELOCITY * MAX_PWM;
        double err_r_pwm = (err_r / TICKS_PER_REV * umfang)/MAX_VELOCITY * MAX_PWM;
     
        integral_l += err_l_pwm * dt;
        integral_r += err_r_pwm * dt;

        int16_t pwm_l = (Kp * err_l_pwm) + (Ki * integral_l);
        int16_t pwm_r = (Kp * err_r_pwm) + (Ki * integral_r);

        std::cout << pwm_l << " " << pwm_r << "|Error L: " << err_l_pwm << " Error R: " << err_r_pwm << std::endl;

        std::chrono::duration<double> time = std::chrono::steady_clock::now() - start_time;
        dataFile << pwm_l << ";" << pwm_r << "\n";

        // Limitieren
        if (pwm_l > MAX_PWM) pwm_l = MAX_PWM; else if (pwm_l < -MAX_PWM) pwm_l = -MAX_PWM;
        if (pwm_r > MAX_PWM) pwm_r = MAX_PWM; else if (pwm_r < -MAX_PWM) pwm_r = -MAX_PWM;

        // std::cout << pwm_l << " " << pwm_r << "|Error L: " << err_l_pwm << " Error R: " << err_r_pwm << std::endl;

        // Schreiben
        dxl.syncWritePWM(pwm_l,pwm_r);

        // Werte für nächsten Durchlauf speichern
        last_pos_l = pos_l;
        last_pos_r = pos_r;

        std::this_thread::sleep_for(std::chrono::milliseconds(iteration_time_ms));
    }

    // Am Ende Motoren aus
    dxl.syncWritePWM(0, 0);
    dataFile.close();
    std::cout << "Datei pwm_log.csv wurde erstellt." << std::endl;
    return true;
}

int main() {
    drive(0.15, 0.15); //max 0.206 m/s

    return 0;
}
