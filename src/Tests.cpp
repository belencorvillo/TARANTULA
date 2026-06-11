#include "Tests.h"
#include "Tarantula.h"
#include "SteadyWinMotor.h"
#include <iostream>
#include <cmath>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Tests::Tests(Tarantula& robot) : robot_(robot) {}

Tests::~Tests() {
    stopTelemetry();
}

void Tests::tickTelemetry()
{
    // Si la telemetría está activa, interrogar por el torque de las articulaciones 2 y 3 de todas las patas
    if (telemetry_active_.load(std::memory_order_relaxed)) {
        for (int i = 0; i < 4; ++i) {
            if (robot_.legs_[i]) {
                MWGetTorques(0, robot_.legs_[i]->id() * 10 + 2);
                MWGetTorques(0, robot_.legs_[i]->id() * 10 + 3);
            }
        }
    }

    // Escribir fila de telemetría si está activa
    if (telemetry_active_.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        if (telemetry_file_.is_open()) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - telemetry_start_time_).count();

            telemetry_file_ << elapsed_ms;
            for (int i = 0; i < 4; ++i) {
                if (robot_.legs_[i]) {
                    for (int joint = 1; joint <= 3; ++joint) {
                        float target = robot_.legs_[i]->getJointTarget(joint);
                        float actual = robot_.legs_[i]->getJointAngle(joint);
                        float error = target - actual;
                        float torque = robot_.legs_[i]->getJointTorque(joint);
                        telemetry_file_ << "," << target << "," << actual << "," << error << "," << torque;
                    }
                } else {
                    for (int joint = 1; joint <= 3; ++joint) {
                        telemetry_file_ << ",0,0,0,0";
                    }
                }
            }
            telemetry_file_ << "\n";
            telemetry_sample_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void Tests::startTelemetry(const std::string& filename)
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    if (telemetry_active_.load()) return;

    telemetry_file_.open(filename);
    if (!telemetry_file_.is_open()) {
        std::cerr << "⚠️ [Telemetry] Error al abrir el archivo: " << filename << "\n";
        return;
    }

    // Cabecera del CSV
    telemetry_file_ << "timestamp_ms";
    for (int leg = 1; leg <= 4; ++leg) {
        for (int joint = 1; joint <= 3; ++joint) {
            telemetry_file_ << ",leg" << leg << "_j" << joint << "_target_deg"
                            << ",leg" << leg << "_j" << joint << "_actual_deg"
                            << ",leg" << leg << "_j" << joint << "_error_deg"
                            << ",leg" << leg << "_j" << joint << "_torque_nm";
        }
    }
    telemetry_file_ << "\n";

    telemetry_start_time_ = std::chrono::steady_clock::now();
    telemetry_sample_count_.store(0);
    telemetry_active_.store(true);
    std::cout << "🚀 [Telemetry] Grabacion iniciada en: " << filename << "\n";
}

void Tests::stopTelemetry()
{
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    if (!telemetry_active_.load()) return;

    telemetry_active_.store(false);
    if (telemetry_file_.is_open()) {
        telemetry_file_.close();
    }
    std::cout << "⏹️ [Telemetry] Grabacion detenida. Muestras guardadas: " 
              << telemetry_sample_count_.load() << "\n";
}

bool Tests::isTelemetryActive() const
{
    return telemetry_active_.load(std::memory_order_relaxed);
}

uint32_t Tests::getTelemetrySampleCount() const
{
    return telemetry_sample_count_.load(std::memory_order_relaxed);
}

void Tests::startCircleIKValidation(int leg_id)
{
    if (robot_.sequence_active_.load()) return;
    robot_.sequence_active_.store(true);
    std::thread(&Tests::runCircleIKValidationSequence, this, leg_id).detach();
}

void Tests::runCircleIKValidationSequence(int leg_id)
{
    if (leg_id < 1 || leg_id > 4) {
        robot_.sequence_active_.store(false);
        return;
    }

    Leg* leg = robot_.legs_[leg_id - 1];
    if (!leg) {
        robot_.sequence_active_.store(false);
        return;
    }

    // 1. Activar la pata si no lo está
    if (!leg->isEnabled()) {
        leg->enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 2. Iniciar telemetría automáticamente con un nombre fijo
    startTelemetry("validacion_ik_circulo.csv");

    // 3. Obtener posición actual del pie
    Eigen::Vector3d p_start = leg->getCurrentFootPosition();

    // Centro del círculo y radio (en metros)
    double xc = 0.350;
    double yc = 0.0;
    double zc = 0.080;
    double R = 0.030;

    // Primer punto del círculo (theta = 0)
    double x_target_start = xc;
    double y_target_start = yc + R;
    double z_target_start = zc;

    // Mover suavemente la pata al inicio del círculo (LERP en 2 segundos, 100 pasos de 20ms)
    int lerp_steps = 100;
    for (int i = 0; i < lerp_steps && robot_.sequence_active_.load(); ++i) {
        double t = static_cast<double>(i) / lerp_steps;
        double x = p_start.x() + (x_target_start - p_start.x()) * t;
        double y = p_start.y() + (y_target_start - p_start.y()) * t;
        double z = p_start.z() + (z_target_start - p_start.z()) * t;

        leg->goToPosition(x, y, z, 3, 4, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Esperar 1 segundo en el punto de inicio para estabilizar
    for (int i = 0; i < 50 && robot_.sequence_active_.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 4. Realizar la trayectoria circular (2 vueltas, 5s por vuelta = 10s, 500 pasos de 20ms)
    int circle_steps = 500;
    double T = 5.0; // periodo en segundos
    double dt = 0.02; // paso de tiempo en segundos
    for (int i = 0; i < circle_steps && robot_.sequence_active_.load(); ++i) {
        double time = i * dt;
        double theta = (2.0 * M_PI / T) * time;

        double x = xc;
        double y = yc + R * std::cos(theta);
        double z = zc + R * std::sin(theta);

        leg->goToPosition(x, y, z, 3, 4, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Esperar 1 segundo
    for (int i = 0; i < 50 && robot_.sequence_active_.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 5. Retornar suavemente a la posición de inicio (apoyada en el suelo)
    Eigen::Vector3d p_end = p_start;
    Eigen::Vector3d p_current(xc, yc + R, zc);

    for (int i = 0; i < lerp_steps && robot_.sequence_active_.load(); ++i) {
        double t = static_cast<double>(i) / lerp_steps;
        double x = p_current.x() + (p_end.x() - p_current.x()) * t;
        double y = p_current.y() + (p_end.y() - p_current.y()) * t;
        double z = p_current.z() + (p_end.z() - p_current.z()) * t;

        leg->goToPosition(x, y, z, 3, 4, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 6. Detener telemetría automáticamente
    stopTelemetry();

    robot_.sequence_active_.store(false);
}

void Tests::startBodyCircleValidation()
{
    if (robot_.sequence_active_.load()) return;
    robot_.sequence_active_.store(true);
    std::thread(&Tests::runBodyCircleValidationSequence, this).detach();
}

void Tests::runBodyCircleValidationSequence()
{
    // 1. Iniciar telemetría automáticamente
    startTelemetry("validacion_ik_cuerpo.csv");

    // 2. Mover el cuerpo suavemente al inicio del círculo (LERP de 2 segundos de (0,0,0) a (Rc, 0, 0))
    double Rc = 0.040; // 40 mm de radio
    int lerp_steps = 100;
    for (int i = 0; i < lerp_steps && robot_.sequence_active_.load(); ++i) {
        double t = static_cast<double>(i) / lerp_steps;
        double dx = Rc * t;
        robot_.setBodyPose(dx, 0.0, 0.0, 0.0, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Esperar 1 segundo para estabilizar
    for (int i = 0; i < 50 && robot_.sequence_active_.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 3. Describir 2 círculos completos (2 vueltas, 5s por vuelta = 10s total, 500 pasos de 20ms)
    int circle_steps = 500;
    double T = 5.0; // periodo en segundos
    double dt = 0.02; // paso de tiempo en segundos
    for (int i = 0; i < circle_steps && robot_.sequence_active_.load(); ++i) {
        double time = i * dt;
        double theta = (2.0 * M_PI / T) * time;

        double dx = Rc * std::cos(theta);
        double dy = Rc * std::sin(theta);

        robot_.setBodyPose(dx, dy, 0.0, 0.0, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Esperar 1 segundo
    for (int i = 0; i < 50 && robot_.sequence_active_.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 4. Retornar suavemente al centro (0,0,0) (LERP de 2 segundos de (Rc, 0, 0) a (0,0,0))
    for (int i = 0; i < lerp_steps && robot_.sequence_active_.load(); ++i) {
        double t = static_cast<double>(i) / lerp_steps;
        double dx = Rc * (1.0 - t);
        robot_.setBodyPose(dx, 0.0, 0.0, 0.0, 0.0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 5. Detener telemetría
    stopTelemetry();

    robot_.sequence_active_.store(false);
}

void Tests::startISO9283Repeatability(int leg_id)
{
    if (robot_.sequence_active_.load()) return;
    robot_.sequence_active_.store(true);
    std::thread(&Tests::runISO9283RepeatabilitySequence, this, leg_id).detach();
}

void Tests::runISO9283RepeatabilitySequence(int leg_id)
{
    if (leg_id < 1 || leg_id > 4) {
        robot_.sequence_active_.store(false);
        return;
    }

    Leg* leg = robot_.legs_[leg_id - 1];
    if (!leg) {
        robot_.sequence_active_.store(false);
        return;
    }

    // 1. Activar la pata si no lo está
    if (!leg->isEnabled()) {
        leg->enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 2. Iniciar telemetría
    startTelemetry("repetibilidad_iso9283.csv");

    // 3. Obtener la posición inicial actual de la pata (home)
    Eigen::Vector3d p_start = leg->getCurrentFootPosition();

    // Posición del punto objetivo A
    double xa = 0.350;
    double ya = 0.0;
    double za = 0.080;

    int lerp_steps = 100; // 2 segundos
    int wait_steps_A = 100; // 2 segundos
    int wait_steps_home = 50; // 1 segundo

    for (int cycle = 0; cycle < 10 && robot_.sequence_active_.load(); ++cycle) {
        // A. Movimiento LERP suave desde home actual (p_start) a punto objetivo A
        for (int i = 0; i < lerp_steps && robot_.sequence_active_.load(); ++i) {
            double t = static_cast<double>(i) / lerp_steps;
            double x = p_start.x() + (xa - p_start.x()) * t;
            double y = p_start.y() + (ya - p_start.y()) * t;
            double z = p_start.z() + (za - p_start.z()) * t;

            leg->goToPosition(x, y, z, 3, 4, 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // B. Esperar/Permanencia en el punto objetivo A (2 segundos)
        for (int i = 0; i < wait_steps_A && robot_.sequence_active_.load(); ++i) {
            leg->goToPosition(xa, ya, za, 3, 4, 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // C. Movimiento LERP suave de retorno desde A hacia p_start (home)
        for (int i = 0; i < lerp_steps && robot_.sequence_active_.load(); ++i) {
            double t = static_cast<double>(i) / lerp_steps;
            double x = xa + (p_start.x() - xa) * t;
            double y = ya + (p_start.y() - ya) * t;
            double z = za + (p_start.z() - za) * t;

            leg->goToPosition(x, y, z, 3, 4, 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // D. Esperar/Permanencia en p_start (1 segundo)
        for (int i = 0; i < wait_steps_home && robot_.sequence_active_.load(); ++i) {
            leg->goToPosition(p_start.x(), p_start.y(), p_start.z(), 3, 4, 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    // 4. Detener telemetría
    stopTelemetry();

    robot_.sequence_active_.store(false);
}
