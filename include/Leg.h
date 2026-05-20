#pragma once
#include <cmath>
#include <utility>
#include "MotorController.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <Eigen/Dense>

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

    
    bool goToPosition(double x, double y, double z, int stiffness_q1 = 3, int stiffness_q2 = 4, int stiffness_q3 = 4); //para cuando tenga la cinemática inversa
    void moveJoint(int joint, float pos_deg, int stiffness);
    void waitUntilSettled(const std::atomic<bool>& sequence_active,
                          float tolerance_deg = 10.0f) const;

   
    bool  isGrounded()             const;  // para cuando implemente los sensores de efecto Hall
    float getJointAngle(int joint) const;  
    Eigen::Vector3d getCurrentFootPosition() const; // con cinemática directa

    
    int id() const { return leg_id_; }

private:
    int              leg_id_;
    MotorController motor_[3];  // [0]=q1  [1]=q2  [2]=q3

    
    JointAngles solveIK(double x, double y, double z)                           const;
    bool        isWithinReach(double s, double d)                               const;
    bool        isWithinJointLimits(const JointAngles& angles)                  const;

    
    Eigen::Vector3d forwardKinematics(double q1, double q2, double q3)                     const;

    //aplica los ángulos a los motores
    void applyAngles(const JointAngles& angles, int stiffness_q1 = 3, int stiffness_q2 = 4, int stiffness_q3 = 4);


};
