    // feet_captured_ = false;  //esta variable la utilizarÃ© para cuando hago lo de la cinemÃ¡tica inversa// feet_captured_ = false;  //esta variable la utilizarÃ© para cuando hago lo de la cinemÃ¡tica inversa
#include "Tarantula.h"
#include "Config.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#endif

static const std::vector<uint32_t> MOTOR_IDS = {
    11, 12, 13,   // Pata 1
    21, 22, 23,   // Pata 2
    31, 32, 33,   // Pata 3
    41, 42, 43    // Pata 4
};

Tarantula::Tarantula(WaveshareInterface& comm)
    : comm_(comm)
{
    createLegs();
}

Tarantula::~Tarantula()
{
    stop();
    for (Leg* leg : legs_) delete leg;
}

void Tarantula::createLegs()
{
    for (int n = 1; n <= 4; ++n) {
        legs_[n - 1] = new Leg(n);
    }
}

void Tarantula::start()
{
    if (running_.load()) return;
    running_.store(true);

    rx_thread_      = std::thread(&Tarantula::rxLoop,      this);
    control_thread_ = std::thread(&Tarantula::controlLoop, this);
}

void Tarantula::stop()
{
    sequence_active_.store(false);
    running_.store(false);

    if (rx_thread_.joinable())      rx_thread_.join();
    if (control_thread_.joinable()) control_thread_.join();

    for (Leg* leg : legs_) {
        if (leg) leg->sendIdleToAllMotors();
    }
}

static const std::vector<uint32_t> ALLOWED_IDS = { 11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43 };

void Tarantula::enableAllLegs()
{
    // Ejecutamos el encendido en un hilo separado para no bloquear la interfaz gráfica
    std::thread boot_thread([this]() {
        for (Leg* leg : legs_) {
            if (leg) leg->enable();
        }
    });
    boot_thread.detach();
}

void Tarantula::disableAllLegs()
{
    abortSequence();
    for (Leg* leg : legs_) {
        if (leg) leg->disable();
    }
}

void Tarantula::moveLeg(int leg_id, double x, double y, double z)
{
    if (leg_id < 1 || leg_id > 4) return;
    legs_[leg_id - 1]->goToPosition(x, y, z);
}

void Tarantula::moveLegJoint(int leg_id, int joint, float pos_deg, int stiffness)
{
    if (leg_id < 1 || leg_id > 4) return;
    legs_[leg_id - 1]->moveJoint(joint, pos_deg, stiffness);
}

void Tarantula::setBodyPose(double dx, double dy, double dz,
                            double roll, double pitch)
{
    if (!feet_captured_) {
        captureFeetPositions();
    }

    int s1 = 3, s2 = 4, s3 = 4; //rigidez predeterminada
    //si movemos el eje x o y ponemos todos los motores con máxima rigidez para que no deslice
    if (std::abs(dx) > 1e-5 || std::abs(dy) > 1e-5) {
        s1 = 5;
        s2 = 5;
        s3 = 5;
    }

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d local_target = computeFootTargetForLeg(i, dx, dy, dz, roll, pitch);
        legs_[i]->goToPosition(local_target.x(), local_target.y(), local_target.z(), s1, s2, s3);
    }
}

void Tarantula::resetBodyPoseReference()
{
    feet_captured_ = false;
}

void Tarantula::captureFeetPositions()
{
    // Usamos los ángulos de la posición de inicio de los comandos de cinemática inversa
    double ref_q1 = 0.0;
    double ref_q2 = 24.0 * M_PI / 180.0;
    double ref_q3 = -100.0 * M_PI / 180.0;

    //Longitudes entre los motores de las patas
    double L1 = 0.08;
    double L2 = 0.19;
    double L3 = 0.225;

    // Posición del extremo de la pata respecto su referencia local (eje q1)
    double r = L1 + L2 * std::cos(ref_q2) + L3 * std::cos(ref_q2 + ref_q3);
    double x_local = r * std::cos(ref_q1);
    double y_local = r * std::sin(ref_q1);
    double z_local = L2 * std::sin(ref_q2) + L3 * std::sin(ref_q2 + ref_q3);

    Eigen::Vector3d p_local(x_local, y_local, z_local);

    for (int i = 0; i < 4; ++i) {
        //Las patas están a 90 grados unas de otras
        double theta = 0.0;
        if      (i == 0) theta = -M_PI / 4.0;
        else if (i == 1) theta =  M_PI / 4.0;
        else if (i == 2) theta =  3.0 * M_PI / 4.0;
        else if (i == 3) theta = -3.0 * M_PI / 4.0;

        double R_hip = 0.15; // 15cm radius
        double x_off = R_hip * std::cos(theta);
        double y_off = R_hip * std::sin(theta);

        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);

        //Coordenadas del extremo de la pata en el sistema de referencia del cuerpo (Ecuaciones de rotación y traslación)
        double x_foot_body = p_local.x() * cos_t - p_local.y() * sin_t + x_off;
        double y_foot_body = p_local.x() * sin_t + p_local.y() * cos_t + y_off;
        double z_foot_body = p_local.z();

        initial_feet_positions_[i] = Eigen::Vector3d(x_foot_body, y_foot_body, z_foot_body);
    }
    feet_captured_ = true;
}

Eigen::Vector3d Tarantula::computeFootTargetForLeg(int idx, double dx, double dy, double dz,
                                         double roll, double pitch) const
{
    if (!feet_captured_) return Eigen::Vector3d::Zero();

    // Posición de pie inicial respecto del cuerpo
    Eigen::Vector3d p_global = initial_feet_positions_[idx];

    // Rotación y traslación del cuerpo
    // R_body = R_y(roll) * R_x(pitch)
    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);

    Eigen::Matrix3d R_body;
    R_body << cr,  sr*sp,  sr*cp,
              0.0,    cp,    -sp,
             -sr,  cr*sp,  cr*cp;

    Eigen::Vector3d T(dx, dy, dz);

    //Siendo P_body dónde se encuentra el pie respecto al nuevo cuerpo movido
    // P_body = R_body^T * (P_global - T)
    Eigen::Vector3d p_body = R_body.transpose() * (p_global - T);

    // Cambiar de sistemas de coordenadas del cuerpo a sistemas de coordenadas de la pata 
    double theta = 0.0;
    if      (idx == 0) theta = -M_PI / 4.0;
    else if (idx == 1) theta =  M_PI / 4.0;
    else if (idx == 2) theta =  3.0 * M_PI / 4.0;
    else if (idx == 3) theta = -3.0 * M_PI / 4.0;

    double R_hip = 0.15; 
    double x_off = R_hip * std::cos(theta);
    double y_off = R_hip * std::sin(theta);

    
    double x_rel = p_body.x() - x_off;
    double y_rel = p_body.y() - y_off;
    double z_rel = p_body.z();

    // Rotación y traslación
    double x_local = x_rel * std::cos(theta) + y_rel * std::sin(theta);
    double y_local = -x_rel * std::sin(theta) + y_rel * std::cos(theta);
    double z_local = z_rel;

    return Eigen::Vector3d(x_local, y_local, z_local);
}



void Tarantula::standUp()
{
    abortSequence();
    sequence_active_.store(true);
    std::thread(&Tarantula::runStandUpSequence, this).detach();
}

void Tarantula::sitDown()
{
    abortSequence();
    sequence_active_.store(true);
    std::thread(&Tarantula::runSitDownSequence, this).detach();
}

void Tarantula::abortSequence()
{
    sequence_active_.store(false);
}

void Tarantula::runStandUpSequence()
{
    for (Leg* leg : legs_) {
        leg->moveJoint(1, 0.0f, 3);
        leg->moveJoint(2, 24.0f, 4);
        leg->moveJoint(3, -100.0f, 4);
    }
    sequence_active_.store(false);
}

void Tarantula::runSitDownSequence()
{
    // Mandar todas las articulaciones a 0° con rigidez 4, simultáneamente
    for (Leg* leg : legs_) {
        leg->moveJoint(1, 0.0f, 3);
        leg->moveJoint(2, 0.0f, 4);
        leg->moveJoint(3, 0.0f, 4);
    }
    sequence_active_.store(false);
}

void Tarantula::sendJointToAllLegsAndWait(int joint, float target_deg, int stiffness)
{
    if (!sequence_active_.load()) return;

    for (Leg* leg : legs_)
        leg->moveJoint(joint, target_deg, stiffness);

    for (Leg* leg : legs_)
        leg->waitUntilSettled(sequence_active_);
}


Leg& Tarantula::getLeg(int leg_id)
{
    return *legs_[leg_id - 1];
}

bool Tarantula::isAnyMotorOnline() const
{
    for (Leg* leg : legs_) {
        if (leg && leg->isAnyMotorOnline()) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
//  HILO RX — lectura de tramas CAN
// ─────────────────────────────────────────────────────────────

void Tarantula::rxLoop()
{
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif

    uint32_t               can_id;
    std::vector<uint8_t>   data;

    while (running_.load(std::memory_order_relaxed)) {
        processIncomingFrames(can_id, data);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void Tarantula::processIncomingFrames(uint32_t& can_id, std::vector<uint8_t>& data)
{
    while (comm_.receive_can_frame(can_id, data)) {
        uint8_t motor_id = can_id >> 5;
        int leg_id = motor_id / 10;
        if (leg_id >= 1 && leg_id <= 4) {
            legs_[leg_id - 1]->handleCanFrame(can_id, data);
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  HILO DE CONTROL — lazo a 100 Hz
// ─────────────────────────────────────────────────────────────

void Tarantula::controlLoop()
{
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif

    auto     last_tick   = std::chrono::steady_clock::now();
    uint64_t cycle_count = 0;

    while (running_.load(std::memory_order_relaxed)) {
        if (isTimeForNextTick(last_tick)) {
            ++cycle_count;
            tickAllLegs(cycle_count);
            last_tick = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

bool Tarantula::isTimeForNextTick(const std::chrono::steady_clock::time_point& last) const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last).count() >= SEND_FREQUENCY;
}

void Tarantula::tickAllLegs(uint64_t cycle_count)
{
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    for (Leg* leg : legs_) {
        if (leg) leg->tick(now_ms, cycle_count);
    }
}




