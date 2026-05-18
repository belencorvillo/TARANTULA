#include "Leg.h"
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>


Leg::Leg(int leg_id)
    : leg_id_(leg_id)
    , motor_{
        {static_cast<uint8_t>(leg_id * 10 + 1), false},
        {static_cast<uint8_t>(leg_id * 10 + 2), true},  // Motor X2 (fémur) invertido mecánicamente
        {static_cast<uint8_t>(leg_id * 10 + 3), false}
      }
{
}  

Leg::~Leg()
{
}


void Leg::enable()
{
    for (int j = 0; j < 3; ++j) {
        motor_[j].enableSafely(3);
    }
}

void Leg::disable()
{
    for (int j = 0; j < 3; ++j) {
        MWSetAxisState(0, motor_[j].node_id, MW_AXIS_STATE_IDLE);
        motor_[j].active.store(false);
        motor_[j].is_trap_traj.store(false);
    }
}

bool Leg::isEnabled() const
{
    // La pata se considera habilitada si al menos un motor está activo
    for (int j = 0; j < 3; ++j)
        if (motor_[j].active.load(std::memory_order_relaxed))
            return true;
    return false;
}

bool Leg::extendToPosition(double x, double y, double z)
{
    //ESTO LO USAREMOS CUANDO TENGA LA CINEMÁTICA INVERSA
    JointAngles angles = solveIK(x, y, z);
    if (!angles.valid) return false;
    applyAngles(angles);
    return true;
}

void Leg::moveJoint(int joint, float pos_deg, int stiffness)
{
    if (joint < 1 || joint > 3) return;
    MotorController& m = motor_[joint - 1];

    m.setTarget(pos_deg * static_cast<float>(M_PI / 180.0), stiffness);
}

void Leg::waitUntilSettled(const std::atomic<bool>& sequence_active,
                           float tolerance_deg) const
{
    bool all_settled = false;
    while (!all_settled && sequence_active.load()) {
        all_settled = true;
        for (int j = 0; j < 3; ++j) {
            if (!motor_[j].isSettled(tolerance_deg)) {
                all_settled = false;
                break;
            }
        }
        if (!all_settled)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (sequence_active.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
}


bool Leg::isGrounded() const
{
    // Para cuando tengamos implementados los sensores de efecto hall
    return true;
}

float Leg::getJointAngle(int joint) const
{
    if (joint < 1 || joint > 3) return 0.0f;
    return motor_[joint - 1].last_known_pos.load(std::memory_order_relaxed)
           * static_cast<float>(180.0 / M_PI);
}


// COMPLETAR CON CINEMÁTICA DIRECTA
// Vec3 Leg::getCurrentFootPosition() const
// {
//     double q1 = motor_[0] ? motor_[0]->last_known_pos.load() : 0.0;
//     double q2 = motor_[1] ? motor_[1]->last_known_pos.load() : 0.0;
//     double q3 = motor_[2] ? motor_[2]->last_known_pos.load() : 0.0;
//     return forwardKinematics(q1, q2, q3);
// }


JointAngles Leg::solveIK(double x, double y, double z) const
{
    // COMPLETAR CON CINEMÁTICA INVERSA
    return {};
}

JointAngles Leg::solveOneBranch(double q1, double s, double d, bool knee_out) const
{
    // COMPLETAR CON CINEMÁTICA INVERSA
    return {};
}

bool Leg::isWithinReach(double s, double d) const
{
    // COMPLETAR CON CINEMÁTICA INVERSA
    return false;
}

bool Leg::isWithinJointLimits(const JointAngles& a) const
{
    // COMPLETAR CON CINEMÁTICA INVERSA
    return false;
}


Vec3 Leg::forwardKinematics(double q1, double q2, double q3) const
{
    // COMPLETAR CON CINEMÁTICA DIRECTA
    return {};
}


void Leg::applyAngles(const JointAngles& angles)
{
    double qs[3] = { angles.q1, angles.q2, angles.q3 };
    int stiffnesses[3] = { 3, 3, 4 };

    for (int i = 0; i < 3; ++i) {
        if (motor_[i].active.load(std::memory_order_relaxed)) {
            motor_[i].setTarget(static_cast<float>(qs[i]), stiffnesses[i]);
        }
    }
}

void Leg::enableJoint(int joint)
{
    if (joint >= 1 && joint <= 3) {
        motor_[joint - 1].enableSafely(3);
    }
}

void Leg::tick(int64_t now_ms, uint64_t cycle_count)
{
    for (int j = 0; j < 3; ++j) {
        motor_[j].tick(now_ms, cycle_count);
    }
}

bool Leg::isAnyMotorOnline() const
{
    for (int j = 0; j < 3; ++j) {
        if (motor_[j].motor_online.load(std::memory_order_relaxed))
            return true;
    }
    return false;
}

void Leg::sendIdleToAllMotors()
{
    for (int j = 0; j < 3; ++j) {
        MWSetAxisState(0, motor_[j].node_id, MW_AXIS_STATE_IDLE);
    }
}

void Leg::handleCanFrame(uint32_t can_id, const std::vector<uint8_t>& data)
{
    uint8_t motor_id = can_id >> 5;
    int joint_idx = (motor_id % 10) - 1;
    if (joint_idx >= 0 && joint_idx < 3) {
        MotorController& ctrl = motor_[joint_idx];
        ctrl.motor_online.store(true, std::memory_order_relaxed);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        ctrl.last_msg_time_ms.store(now_ms, std::memory_order_relaxed);

        if (data.size() == 8) {
            MWReceiver(0, can_id, const_cast<uint8_t*>(data.data()));
            MW_CMD_ID cmd = static_cast<MW_CMD_ID>(can_id & 0x1F);
            if (cmd == MW_MIT_CONTROL_CMD) {
                ctrl.update_from_mit_frame();
            } else if (cmd == MW_GET_ENCODER_ESTIMATES_CMD) {
                ctrl.update_from_encoder_frame();
            }
        }
    }
}

