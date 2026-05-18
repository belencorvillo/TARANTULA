#pragma once
#include <array>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include "Leg.h"
#include "WaveshareInterface.h"
#include <Eigen/Dense>

extern const double SEND_FREQUENCY;


class Tarantula {
public:
    explicit Tarantula(WaveshareInterface& comm);
    ~Tarantula();

   
    void start();           
    void stop();            
    void enableAllLegs();   
    void disableAllLegs();  

    void moveLeg(int leg_id, double x, double y, double z); //para cuando haga la cinemática inversa
    void moveLegJoint(int leg_id, int joint, float pos_deg, int stiffness);

    void setBodyPose(double dx, double dy, double dz, double roll, double pitch);
    void resetBodyPoseReference();

    void standUp();
    void sitDown();
    void abortSequence();  

    Leg& getLeg(int leg_id);                 
    bool isAnyMotorOnline() const;
    bool isRunning() const { return running_.load(); }

private:
   
    WaveshareInterface& comm_;
    std::array<Leg*, 4> legs_;  
    bool feet_captured_{ false };
    std::array<Eigen::Vector3d, 4> initial_feet_positions_;
    
    std::atomic<bool> running_{ false };
    std::thread       rx_thread_;
    std::thread       control_thread_;
    std::atomic<bool> sequence_active_{ false };

    void rxLoop();
    void controlLoop();

   
    void createLegs();

    // Captura las posiciones actuales de los pies mediante FK
    void captureFeetPositions();
    Eigen::Vector3d computeFootTargetForLeg(int leg_idx, double dx, double dy, double dz,
                                         double roll, double pitch) const;

    void runStandUpSequence();
    void runSitDownSequence();
    void sendJointToAllLegsAndWait(int joint, float target_deg, int stiffness);

    // Sub-funciones del hilo RX
    void processIncomingFrames(uint32_t& can_id, std::vector<uint8_t>& data);

    // Sub-funciones del hilo de control
    bool isTimeForNextTick(const std::chrono::steady_clock::time_point& last) const;
    void tickAllLegs(uint64_t cycle_count);
};
