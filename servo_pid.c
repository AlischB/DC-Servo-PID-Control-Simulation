#include <stdio.h>

// Motor Parametreleri
#define R 1.0
#define L 0.5
#define J 0.01
#define B 0.1
#define K 0.01
#define DT 0.001 // 1 ms örnekleme zamanı (1 kHz)

// PID Yapısı 
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double prev_error;
    double out_min;
    double out_max;
} PID_Controller;

double PID_Compute(PID_Controller* pid, double setpoint, double measured) {
    double error = setpoint - measured;

    // Integral (Anti-windup sınırlamalı)
    pid->integral += error * DT;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    // Türev (Derivative)
    double derivative = (error - pid->prev_error) / DT;
    pid->prev_error = error;

    // Toplam Çıkış (Motor Voltajı)
    double output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Voltaj Saturasyonu (Örn: 24V Besleme Sınırı)
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}

int main() {
    FILE* fp = fopen("servo_log.csv", "w");
    fprintf(fp, "Time,Setpoint,Angle,Voltage\n");

    PID_Controller pid = { 18.0, 2.5, 1.2, 0.0, 0.0, -24.0, 24.0 };

    double target_angle = 45.0; // Hedef: 45 derece kontrol yüzeyi açısı
    double angle = 0.0, omega = 0.0, current = 0.0;

    // 2 Saniyelik Simülasyon (2000 adım)
    for (int step = 0; step < 2000; step++) {
        double time = step * DT;
        double voltage = PID_Compute(&pid, target_angle, angle);

        // Euler Yöntemi ile Fiziksel Motor Dinamiği
        double d_current = (voltage - R * current - K * omega) / L;
        double d_omega = (K * current - B * omega) / J;

        current += d_current * DT;
        omega += d_omega * DT;
        angle += omega * DT;

        fprintf(fp, "%.4f,%.2f,%.4f,%.4f\n", time, target_angle, angle, voltage);
    }

    fclose(fp);
    printf("Simulasyon tamamlandi. Veriler servo_log.csv dosyasina yazildi.\n");
    return 0;
}