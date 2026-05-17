#pragma once
#include <cmath>
#include <utility>
#include "MotorController.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

struct JointAngles {
    double q1 = 0.0;   
    double q2 = 0.0;   
    double q3 = 0.0;   
    bool   valid = false;
};

class Leg {
public:
   
    Leg(int leg_id);
    ~Leg();

   
    void enable();          
    void disable();         
    bool isEnabled() const;
    void enableJoint(int joint);
    void tick(int64_t now_ms, uint64_t cycle_count);
    bool isAnyMotorOnline() const;
    void sendIdleToAllMotors();
    void handleCanFrame(uint32_t can_id, const std::vector<uint8_t>& data);

    
    bool extendToPosition(double x, double y, double z); //para cuando tenga la cinemática inversa
    void moveJoint(int joint, float pos_deg, int stiffness);
    void waitUntilSettled(const std::atomic<bool>& sequence_active,
                          float tolerance_deg = 10.0f) const;

   
    bool  isGrounded()             const;  // para cuando implemente los sensores de efecto Hall
    float getJointAngle(int joint) const;  
    // Vec3  getCurrentFootPosition() const; // con cinemática directa

    
    int id() const { return leg_id_; }

private:
    int              leg_id_;
    MotorController* motor_[3];  // [0]=q1  [1]=q2  [2]=q3

    
    JointAngles solveIK(double x, double y, double z)                           const;
    //siendo s la distancia horizontal y d la vertical
    JointAngles solveOneBranch(double q1, double s, double d, bool knee_out)    const;
    bool        isWithinReach(double s, double d)                               const;
    bool        isWithinJointLimits(const JointAngles& angles)                  const;

    
    Vec3 forwardKinematics(double q1, double q2, double q3)                     const;

    //aplica los ángulos a los motores
    void applyAngles(const JointAngles& angles);


};
