#include "MotorController.h"
#include <cstring>
#include <thread>
#include <chrono>

WaveshareInterface* g_comm = nullptr; //puntero global que apuntará a mi Waveshare (lo iniciamos vacío hasta conectarlo)
MotorController* g_controllers[128] = { nullptr }; //motores conectados (el máximo es 128)

MotorController::MotorController(uint8_t id, bool reversed) : node_id(id), reversed(reversed) {
    memset(&motorData, 0, sizeof(MW_MOTOR_DATA)); //limpiamos la memoria

    MW_MOTOR_ACCESS_INFO access_info;
    access_info.busId = 0; //usamos el mismo canal bus para todos los motores
    access_info.nodeId = node_id;
    access_info.motorData = &motorData; //le decimos al motor que escriba los datos del motor (estructura MW_MOTOR_ACCESS_INFO) en motorData

    //función lambda sender:
    access_info.sender = [](uint8_t busId, uint32_t canId, uint8_t* data, uint8_t dataSize) {
        if (g_comm) { //si el waveshare está conectado, ejecuta:
            std::vector<uint8_t> vec(data, data + dataSize); //convertimos el puntero  uint8_t* data a vector para pasárselo a Waveshare
            g_comm->send_can_frame(canId, vec); //le decimos al waveshare que envíe la trama
        }
        };

    access_info.notifier = [](uint8_t, uint8_t, MW_CMD_ID) {}; //no asignamos notifier, leemos todo el rato actualizando los valores last_known

    MWRegisterMotor(access_info); //registramos el motor en la matriz de motores de SteadyWinMotor
    g_controllers[node_id] = this; //guarda una referencia a sí mismo
}

MotorController :: ~MotorController() { g_controllers[node_id] = nullptr; }

// ─────────────────────────────────────────────────────────────
// CONVERSIÓN DE COORDENADAS
// ─────────────────────────────────────────────────────────────

float MotorController::convert_dir(float val) const {
    // Invierte el signo si el motor está invertido mecánicamente
    return reversed ? -val : val;
}

float MotorController::phys_to_logic(float phys_pos) const {
    // La posición física está invertida respecto al cero lógico si reversed es true
    return reversed ? -phys_pos : phys_pos;
}

float MotorController::logic_to_phys(float log_pos) const {
    // La posición lógica se invierte para enviarla al motor si reversed es true
    return reversed ? -log_pos : log_pos;
}

// ─────────────────────────────────────────────────────────────
// FUNCIONES PARA HILO RX
//─────────────────────────────────────────────────────────────

void MotorController::update_from_mit_frame() {
    // Convierte datos del rotor a valores lógicos del sistema y los almacena.
    // La división por GEAR_RATIO pasa de radianes del rotor a radianes del eje de salida.
    float phys_pos    = motorData.motorMIT.pos    / (float)GEAR_RATIO;
    float phys_vel    = motorData.motorMIT.vel    / (float)GEAR_RATIO;
    float phys_torque = motorData.motorMIT.torque;

    float abs_log_pos = phys_to_logic(phys_pos);
    last_known_pos.store(abs_log_pos - pos_offset.load(std::memory_order_relaxed), std::memory_order_relaxed);
    last_known_vel.store(convert_dir(phys_vel), std::memory_order_relaxed);
    last_known_torque.store(convert_dir(phys_torque), std::memory_order_relaxed);
}

void MotorController::update_from_encoder_frame() {
    // Convierte las revoluciones del encoder (rotor) a radianes lógicos del eje de salida.
    float output_revs = motorData.encoderEstimates.encoderPosEstimate / (float)GEAR_RATIO;
    float phys_rads   = output_revs * 2.0f * (float)M_PI;
    float abs_log_pos = phys_to_logic(phys_rads);
    last_known_pos.store(abs_log_pos - pos_offset.load(std::memory_order_relaxed), std::memory_order_relaxed);
}

// ─────────────────────────────────────────────────────────────
// FUNCIONES PARA HILO DE CONTROL
// ─────────────────────────────────────────────────────────────

//En el hilo de control se llama a esta función cada tick para saber si un motor ha dejado de responder
bool MotorController::check_timeout(int64_t now_ms, int64_t timeout_ms) {
    // Solo actúa si el motor estaba online.
    if (!motor_online.load(std::memory_order_relaxed)) return false;

    //last_msg_time_ms indica la última vez que se recibió un mensaje del motor (actualizado en el hilo RX)
    uint64_t last_msg = last_msg_time_ms.load(std::memory_order_relaxed);
    if (now_ms > (int64_t)last_msg && (now_ms - (int64_t)last_msg) > timeout_ms) {
        motor_online.store(false, std::memory_order_relaxed);
        active.store(false, std::memory_order_relaxed);
        return true; // Acaba de pasar a offline
    }
    return false;
}

MW_MIT_CTRL MotorController::step_trajectory() {
    // Calcula un paso del generador de trayectoria trapezoidal.
    // El dt fijo (SEND_FREQUENCY) es intencionado: los parámetros max_vel y max_accel
    // están calibrados para este valor, y el velocity feedforward (mit.vel) hace que
    // el motor interpole suavemente entre comandos consecutivos.
    float current_p = current_traj_pos.load(std::memory_order_relaxed);
    float current_v = current_traj_vel.load(std::memory_order_relaxed);
    float target_p  = target_pos.load(std::memory_order_relaxed);
    float max_v     = traj_max_vel.load(std::memory_order_relaxed);
    float max_a     = traj_max_accel.load(std::memory_order_relaxed);
    float dt        = (float)SEND_FREQUENCY;

    float error = target_p - current_p;
    float dir   = (error > 0.0f) ? 1.0f : -1.0f;

    if (std::abs(error) < 0.005f && std::abs(current_v) < 0.01f) {
        // Suficientemente cerca y casi parados: anclamos al destino
        current_p = target_p;
        current_v = 0.0f;
    } else {
        // Curva de frenado trapezoidal: velocidad máxima limitada por la distancia restante
        float v_brake   = std::sqrt(2.0f * max_a * std::abs(error));
        float v_desired = dir * std::min(max_v, v_brake);

        // Acelerar o frenar suavemente hacia la velocidad deseada
        if      (current_v < v_desired) { current_v += max_a * dt; if (current_v > v_desired) current_v = v_desired; }
        else if (current_v > v_desired) { current_v -= max_a * dt; if (current_v < v_desired) current_v = v_desired; }

        // Integrar velocidad → nueva posición
        current_p += current_v * dt;
    }

    current_traj_pos.store(current_p, std::memory_order_relaxed);
    current_traj_vel.store(current_v, std::memory_order_relaxed);

    MW_MIT_CTRL mit;
    float abs_p  = current_p + pos_offset.load(std::memory_order_relaxed);
    mit.pos      = logic_to_phys(abs_p);
    mit.vel      = convert_dir(current_v);
    mit.kp       = target_kp.load(std::memory_order_relaxed);
    mit.kd       = target_kd.load(std::memory_order_relaxed);
    mit.torque   = convert_dir(target_torque.load(std::memory_order_relaxed));
    return mit;
}

MW_MIT_CTRL MotorController::build_static_mit() {
    // Construye el struct MIT con los targets fijos actuales (sin trayectoria).
    MW_MIT_CTRL mit;
    float abs_p  = target_pos.load(std::memory_order_relaxed) + pos_offset.load(std::memory_order_relaxed);
    mit.pos      = logic_to_phys(abs_p);
    mit.vel      = convert_dir(target_vel.load(std::memory_order_relaxed));
    mit.kp       = target_kp.load(std::memory_order_relaxed);
    mit.kd       = target_kd.load(std::memory_order_relaxed);
    mit.torque   = convert_dir(target_torque.load(std::memory_order_relaxed));
    return mit;
}


void MotorController::enableSafely(int stiffness)
{
    if (!motor_online.load(std::memory_order_relaxed)) return;

    MWSetAxisState(0, node_id, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    target_kp.store(0.0f);
    target_kd.store(0.0f);
    active.store(true);

    // Esperar a que el motor reporte su posición
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Capturar la posición actual como offset (define el "cero lógico")
    pos_offset.store(last_known_pos.load() + pos_offset.load());
    target_pos.store(0.0f);
    current_traj_pos.store(0.0f);

    // Aplicar rigidez
    applyStiffness(stiffness); 
}

void MotorController::applyStiffness(int stiffness)
{
    auto [kp, kd] = stiffnessToGains(stiffness);
    target_kp.store(kp);
    target_kd.store(kd);
}

std::pair<float, float> MotorController::stiffnessToGains(int stiffness)
{
    switch (stiffness) {
    case 1:  return { 5.0f,  0.1f };
    case 2:  return { 10.0f, 0.3f };
    case 3:  return { 20.0f, 0.5f };
    case 4:  return { 35.0f, 1.0f };
    case 5:  return { 50.0f, 1.5f };
    default: return { 20.0f, 0.5f };
    }
}

bool MotorController::isSettled(float tolerance_deg) const
{
    if (!motor_online.load(std::memory_order_relaxed) || !active.load(std::memory_order_relaxed)) {
        return true; // Si está apagado/offline, no bloquea la secuencia
    }
    float current_deg = last_known_pos.load(std::memory_order_relaxed) * static_cast<float>(180.0 / M_PI);
    float target_deg  = target_pos.load(std::memory_order_relaxed) * static_cast<float>(180.0 / M_PI);
    return std::abs(current_deg - target_deg) <= tolerance_deg;
}

void MotorController::setTarget(float pos_rad, int stiffness)
{
    auto [kp, kd] = stiffnessToGains(stiffness);
    target_pos.store(pos_rad);
    target_kp.store(kp);
    target_kd.store(kd);
    is_trap_traj.store(true);
}

void MotorController::tick(int64_t now_ms, uint64_t cycle_count)
{
    check_timeout(now_ms);

    if (!motor_online.load(std::memory_order_relaxed)) {
        if (cycle_count % 50 == (node_id % 50)) {
            MWGetEncoderEstimates(0, node_id);
        }
        return;
    }

    if (active.load(std::memory_order_relaxed)) {
        MW_MIT_CTRL mit = is_trap_traj.load(std::memory_order_relaxed)
                        ? step_trajectory()
                        : build_static_mit();
        MWMitControl(0, node_id, &mit);
    } else {
        MWGetEncoderEstimates(0, node_id);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
