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
    auto start_time = std::chrono::steady_clock::now();

    int drive_time_seconds = 20;
    int iteration_time_ms = 10;
    int iterations = (drive_time_seconds * 1000) / iteration_time_ms; // 20ms Loop-Zeit
    std::cout << "Iterations: " << iterations << std::endl;

      // 1. Zielwerte berechnen (m/s -> Ticks/s)
    double umfang = RAD_DURCHMESSER * PI;
    double target_l = (v_l / umfang) * TICKS_PER_REV;
    double target_r = (v_r / umfang) * TICKS_PER_REV;

    // 2. Initialisierung für den Regler
    int32_t last_pos_l = dxl.readPosition(dxl.DXL_LEFT_ID);
    int32_t last_pos_r = dxl.readPosition(dxl.DXL_RIGHT_ID);

    double Kp = 4;
    double Ki = 0;
    double integral_l = 0, integral_r = 0;
    double dt = iteration_time_ms / 1000.0; // 20ms Loop-Zeit in Sekunden

    std::ofstream dataFile("pwm_log.csv");
    dataFile << "Iteration;PWM_Wert" << std::endl;

    // 3. Fahr-Schleife (Beispiel: 5 Sekunden = 250 Iterationen)
    //for (int i = 0; i < iterations; i++) {
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(drive_time_seconds)) {
        // Positionen lesen
        int32_t pos_l = dxl.readPosition(dxl.DXL_LEFT_ID);
        int32_t pos_r = dxl.readPosition(dxl.DXL_RIGHT_ID);

        // Aktuelle Geschwindigkeit berechnen
        double vel_l = (pos_l - last_pos_l) / dt;
        double vel_r = (pos_r - last_pos_r) / dt;

        // PI-Regler (kp=0.15, ki=0.05)
        double err_l = target_l - vel_l;
        double err_r = target_r - vel_r;

        double err_l_pwm = (err_l / TICKS_PER_REV * umfang)/MAX_VELOCITY * MAX_PWM;
        double err_r_pwm = (err_r / TICKS_PER_REV * umfang)/MAX_VELOCITY * MAX_PWM;

        integral_l += err_l * dt;
        integral_r += err_r * dt;

        double u_l = (Kp * err_l) + (Ki * integral_l);
        double u_r = (Kp * err_r) + (Ki * integral_r);

        double v_l_u = (u_l / TICKS_PER_REV) * umfang; // zurück in m/s
        double v_r_u = (u_r / TICKS_PER_REV) * umfang;

        int pwm_l = v_l_u / MAX_VELOCITY * MAX_PWM;
        int pwm_r = v_r_u / MAX_VELOCITY * MAX_PWM;

        std::cout << pwm_l << " " << pwm_r << "|Error L: " << err_l_pwm << " Error R: " << err_r_pwm << std::endl;

        // Limitieren
        if (pwm_l > MAX_PWM) pwm_l = MAX_PWM; else if (pwm_l < -MAX_PWM) pwm_l = -MAX_PWM;
        if (pwm_r > MAX_PWM) pwm_r = MAX_PWM; else if (pwm_r < -MAX_PWM) pwm_r = -MAX_PWM;

        // std::cout << pwm_l << " " << pwm_r << "|Error L: " << err_l_pwm << " Error R: " << err_r_pwm << std::endl;

        // Schreiben
        dxl.writePWM(dxl.DXL_LEFT_ID, pwm_l);
        dxl.writePWM(dxl.DXL_RIGHT_ID, pwm_r);

        // Werte für nächsten Durchlauf speichern
        last_pos_l = pos_l;
        last_pos_r = pos_r;
    }

    // Am Ende Motoren aus
    dxl.syncWritePWM(0, 0);
    return true;
}

int main() {
    drive(0.15, 0.15); // Vorwärts mit 0.2 m/s

    return 0;
}
