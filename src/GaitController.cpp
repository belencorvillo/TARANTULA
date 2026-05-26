#include "GaitController.h"
#include "Leg.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GaitController::GaitController()
    : active_(false)
    , target_vx_(0.0f)
    , target_vy_(0.0f)
    , phase_(0.0)
    , last_time_ms_(0)
{
}

GaitController::~GaitController()
{
}

void GaitController::start()
{
    active_.store(true);
}

void GaitController::stop()
{
    active_.store(false);
    target_vx_.store(0.0f);
    target_vy_.store(0.0f);
}

bool GaitController::isActive() const
{
    return active_.load();
}

void GaitController::setVelocity(float vx, float vy)
{
    target_vx_.store(vx);
    target_vy_.store(vy);
}

void GaitController::tick(std::array<Leg*, 4>& legs, int64_t now_ms)
{
    if (last_time_ms_ == 0) {
        last_time_ms_ = now_ms;
        return;
    }

    double dt = (now_ms - last_time_ms_) * 0.001;
    last_time_ms_ = now_ms;

    float vx = target_vx_.load();
    float vy = target_vy_.load();
    bool walking = (std::abs(vx) > 0.001f || std::abs(vy) > 0.001f);

    // Si no está activo y la velocidad es 0, y ya estamos parados en pose neutra (fase 0), no hacemos nada.
    if (!active_.load() && !walking && phase_ == 0.0) {
        return;
    }

    // Avanzar la fase de la marcha
    double omega = (2.0 * M_PI) / GAIT_PERIOD;
    phase_ += omega * dt;
    if (phase_ >= 2.0 * M_PI) {
        phase_ = std::fmod(phase_, 2.0 * M_PI);
    }

    // Longitud del paso en X e Y
    double Sx = vx * GAIT_PERIOD;
    double Sy = vy * GAIT_PERIOD;

    // Parejas diagonales:
    // Pareja A: Pata 1 (index 0) y Pata 3 (index 2) -> desfase = 0
    // Pareja B: Pata 2 (index 1) y Pata 4 (index 3) -> desfase = PI
    double offsets[4] = { 0.0, M_PI, 0.0, M_PI };

    for (int i = 0; i < 4; ++i) {
        Leg* leg = legs[i];
        if (!leg) continue;

        double leg_phase = std::fmod(phase_ + offsets[i], 2.0 * M_PI);
        if (leg_phase < 0.0) leg_phase += 2.0 * M_PI;

        double x_rel = 0.0;
        double y_rel = 0.0;
        double z_rel = 0.0;

        if (leg_phase < M_PI) {
            // ─── FASE DE VUELO (SWING) ───
            double sigma = leg_phase / M_PI; // [0, 1)
            x_rel = -Sx / 2.0 + Sx * sigma;
            y_rel = -Sy / 2.0 + Sy * sigma;
            z_rel = SWING_HEIGHT * std::sin(M_PI * sigma);
        } else {
            // ─── FASE DE APOYO (STANCE) ───
            double sigma = (leg_phase - M_PI) / M_PI; // [0, 1)
            x_rel = Sx / 2.0 - Sx * sigma;
            y_rel = Sy / 2.0 - Sy * sigma;
            z_rel = 0.0;
        }

        // Obtener la posición neutra inicial (pose de levantado) en coordenadas del cuerpo
        Eigen::Vector3d p_neutral = leg->getInitialFootPosition();
        
        // Sumar el desplazamiento relativo en coordenadas de cuerpo
        Eigen::Vector3d p_body = p_neutral + Eigen::Vector3d(x_rel, y_rel, z_rel);

        // Comandar la pata usando el método encapsulado de cuerpo (con control directo sin filtro)
        leg->goToBodyPosition(p_body, STIFFNESS_Q1, STIFFNESS_Q2, STIFFNESS_Q3, true, true);
    }

    // ─── ALGORITMO DE FRENADO ULTRA-SUAVE (SOFT SETTLE) ───
    // Si se ha pedido detener la marcha, esperamos a que la fase esté en una zona de apoyo 
    // plano (cerca de 0, PI o 2*PI) para que las patas en el aire aterricen con suavidad,
    // y entonces reseteamos la fase a 0.0 clavando el robot en su pose de levantado perfecta.
    if (!active_.load() && !walking) {
        double diff_0 = std::abs(phase_);
        double diff_pi = std::abs(phase_ - M_PI);
        double diff_2pi = std::abs(phase_ - 2.0 * M_PI);

        if (diff_0 < 0.15 || diff_pi < 0.15 || diff_2pi < 0.15) {
            phase_ = 0.0;
            for (int i = 0; i < 4; ++i) {
                if (legs[i]) {
                    legs[i]->goToBodyPosition(legs[i]->getInitialFootPosition(), STIFFNESS_Q1, STIFFNESS_Q2, STIFFNESS_Q3, true, true);
                }
            }
        }
    }
}
