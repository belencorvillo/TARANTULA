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

bool Leg::goToPosition(double x, double y, double z, int stiffness_q1, int stiffness_q2, int stiffness_q3)
{
    //ESTO LO USAREMOS CUANDO TENGA LA CINEMÁTICA INVERSA
    JointAngles angles = solveIK(x, y, z);
    if (!angles.valid) return false;
    applyAngles(angles, stiffness_q1, stiffness_q2, stiffness_q3);
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


Eigen::Vector3d Leg::getCurrentFootPosition() const
{
    double q1 = motor_[0].last_known_pos.load(std::memory_order_relaxed);
    double q2 = motor_[1].last_known_pos.load(std::memory_order_relaxed);
    double q3 = motor_[2].last_known_pos.load(std::memory_order_relaxed);
    return forwardKinematics(q1, q2, q3);
}


JointAngles Leg::solveIK(double x, double y, double z) const
{

    // Longitudes entre ejes de los motores:
    double L1 = 0.08; //Coxa (q1 y q2)
    double L2 = 0.19;  // Femur (q2 y q3)
    double L3 = 0.225; // Tibia (q3 y efector final)


    JointAngles angles;
    angles.valid = false;

    // Ángulo de q1 (yaw)
    double q1 = std::atan2(y, x); // ec 137, pág 44

    // Proyectamos a plano xy:

    //distancia horizontal desde q1 hasta el punto: 
    double s = std::sqrt(x * x + y * y); // ec 138, pág 44
    //si queremos saber esa distancia desde q2:
    double s_femur = s - L1; //ec 139, pág 44
    //distancia en dirección vertical:
    double d = z; //ec 140, pág 44

    //El punto que queremos alcanzar es (s_femur,d)
    if (!isWithinReach(s_femur, d)) {
        return angles;
    }

    double D2 = s_femur * s_femur + d * d; //ec auxiliar

    
    double cosq3 = (D2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3); //ec 145, pág 45
    cosq3 = std::max(-1.0, std::min(1.0, cosq3)); // Clamp de seguridad indispensable ????
    
    double q3 = std::acos(cosq3);
    // Como queremos que esté en postura arácnida (rodilla arriba), aplicamos el ángulo negativo:
    angles.q3 = -q3;

    // Obtenemos el ángulo q2 por la ley de los cosenos:
    double alpha1 = std::atan2(d, s_femur);
    double cosAlpha2 = (L2 * L2 + D2 - L3 * L3) / (2.0 * L2 * std::sqrt(D2));
    cosAlpha2 = std::max(-1.0, std::min(1.0, cosAlpha2));
    double alpha2 = std::acos(cosAlpha2);

    // Para posición con rodilla arriba:
    angles.q2 = alpha1 + alpha2;
    angles.q1 = q1;
    
    // Mirar si es viable:
    if (isWithinJointLimits(angles)) {
        angles.valid = true;
    }

    return angles;
}



bool Leg::isWithinReach(double s, double d) const
{
    double D = std::sqrt(s * s + d * d);
    // Max reach = 19cm + 22.5cm = 41.5cm = 0.415m
    // Min reach = |19cm - 22.5cm| = 3.5cm = 0.035m
    return (D >= 0.035 && D <= 0.415);
}

bool Leg::isWithinJointLimits(const JointAngles& a) const
{
    double q1_deg = a.q1 * 180.0 / M_PI;
    double q2_deg = a.q2 * 180.0 / M_PI;
    double q3_deg = a.q3 * 180.0 / M_PI;

    // límites
    if (q1_deg < -45.0 || q1_deg > 45.0) return false;
    if (q2_deg < -60.0 || q2_deg > 90.0) return false;
    if (q3_deg < -150.0 || q3_deg > 0.0) return false; //para que siempre sea "rodilla arriba"

    return true;
}


Eigen::Vector3d Leg::forwardKinematics(double q1, double q2, double q3) const
{
    double L1 = 0.08;
    double L2 = 0.19;
    double L3 = 0.225;

    // Fórmulas de cinemática directa (pg documentación de Guille):
    double r = L1 + L2 * std::cos(q2) + L3 * std::cos(q2 + q3); //ec 120, pág 40 (parámetro auxiliar)
    double x = r * std::cos(q1); //ec 119, pag 40
    double y = r * std::sin(q1); //ec 119, pag 40
    double z = L2 * std::sin(q2) + L3 * std::sin(q2 + q3); //ec 119, pag 40

    return Eigen::Vector3d(x, y, z);
}


void Leg::applyAngles(const JointAngles& angles, int stiffness_q1, int stiffness_q2, int stiffness_q3)
{
    double qs[3] = { angles.q1, angles.q2, angles.q3 };
    int stiffnesses[3] = { stiffness_q1, stiffness_q2, stiffness_q3 };

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

