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
    //Para cuando haya calculado la cinemática inversa
    if (leg_id < 1 || leg_id > 4) return;
    legs_[leg_id - 1]->extendToPosition(x, y, z);
}

void Tarantula::moveLegJoint(int leg_id, int joint, float pos_deg, int stiffness)
{
    if (leg_id < 1 || leg_id > 4) return;
    legs_[leg_id - 1]->moveJoint(joint, pos_deg, stiffness);
}

void Tarantula::setBodyPose(double dx, double dy, double dz,
                            double roll, double pitch)
{
    // COMPLETAR CON CINEMÁTICA INVERSA
}

void Tarantula::resetBodyPoseReference()
{
    // feet_captured_ = false;
}

void Tarantula::captureFeetPositions()
{
    // COMPLETAR CON CINEMÁTICA INVERSA
}

Vec3 Tarantula::computeFootTargetForLeg(int idx, double dx, double dy, double dz,
                                         double roll, double pitch) const
{
    // COMPLETAR CON CINEMÁTICA INVERSA
    return {};
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
    leg->moveJoint(2, 20.0f, 4);
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




