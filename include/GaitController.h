#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <Eigen/Dense>

class Leg;

class GaitController {
public:
    GaitController();
    ~GaitController();

    void start();
    void stop();
    bool isActive() const;

    void setVelocity(float vx, float vy);
    void tick(std::array<Leg*, 4>& legs, int64_t now_ms);

private:
    std::atomic<bool> active_{false};
    std::atomic<float> target_vx_{0.0f};
    std::atomic<float> target_vy_{0.0f};

    // Parámetros internos de la marcha
    double phase_{0.0};
    int64_t last_time_ms_{0};

    // Constantes de diseño estables
    static constexpr double GAIT_PERIOD = 1.0;     // Ciclo completo en segundos (1 Hz)
    static constexpr double SWING_HEIGHT = 0.10;    // Altura de vuelo del pie en metros (8 cm)
    
    // Perfiles de rigidez seguros y sintonizados
    static constexpr int STIFFNESS_Q1 = 3; // Kp = 15.0
    static constexpr int STIFFNESS_Q2 = 5; // Kp = 32.0 (perfil 5 optimizado con Kd = 1.8)
    static constexpr int STIFFNESS_Q3 = 5; // Kp = 32.0 (perfil 5 optimizado con Kd = 1.8)
};
