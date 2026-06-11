#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct JointAngles {
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
    bool valid = false;
};

class LegDebug {
public:
    int leg_id_;
    double theta_;
    double x_off_, y_off_;
    double initial_foot_x_, initial_foot_y_, initial_foot_z_;

    static constexpr double R_HIP = 0.15;
    static constexpr double L1 = 0.08;
    static constexpr double L2 = 0.188;
    static constexpr double L3 = 0.211;

    LegDebug(int id, double theta) : leg_id_(id), theta_(theta) {
        x_off_ = R_HIP * std::cos(theta_);
        y_off_ = R_HIP * std::sin(theta_);

        double q1_ideal = 0.0;
        double q2_ideal = 20.0 * M_PI / 180.0;
        double q3_ideal = -100.0 * M_PI / 180.0;

        double r = L1 + L2 * std::cos(q2_ideal) + L3 * std::cos(q2_ideal + q3_ideal);
        double x_local = r * std::cos(q1_ideal);
        double y_local = r * std::sin(q1_ideal);
        double z_local = L2 * std::sin(q2_ideal) + L3 * std::sin(q2_ideal + q3_ideal);

        initial_foot_x_ = x_off_ + (x_local * std::cos(theta_) - y_local * std::sin(theta_));
        initial_foot_y_ = y_off_ + (x_local * std::sin(theta_) + y_local * std::cos(theta_));
        initial_foot_z_ = z_local;
    }

    bool isWithinJointLimits(const JointAngles& a) const {
        double q1_deg = a.q1 * 180.0 / M_PI;
        double q2_deg = a.q2 * 180.0 / M_PI;
        double q3_deg = a.q3 * 180.0 / M_PI;

        if (q1_deg < -90.0 || q1_deg > 90.0) return false;
        if (q2_deg < -100.0 || q2_deg > 100.0) return false;
        if (q3_deg < -150.0 || q3_deg > 30.0) return false; // user's new limits
        return true;
    }

    JointAngles solveIK(double x, double y, double z) {
        JointAngles angles;
        angles.q1 = std::atan2(y, x);
        double s = std::sqrt(x * x + y * y);
        double s_femur = s - L1;
        double d = z;

        double D2 = s_femur * s_femur + d * d;
        double cosq3 = (D2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
        if (cosq3 < -1.0001 || cosq3 > 1.0001) {
            angles.valid = false;
            return angles;
        }
        cosq3 = std::max(-1.0, std::min(1.0, cosq3));
        double q3 = std::acos(cosq3);
        angles.q3 = -q3;

        double alpha1 = std::atan2(d, s_femur);
        double cosAlpha2 = (L2 * L2 + D2 - L3 * L3) / (2.0 * L2 * std::sqrt(D2));
        if (cosAlpha2 < -1.0001 || cosAlpha2 > 1.0001) {
            angles.valid = false;
            return angles;
        }
        cosAlpha2 = std::max(-1.0, std::min(1.0, cosAlpha2));
        double alpha2 = std::acos(cosAlpha2);
        angles.q2 = alpha1 + alpha2;
        
        angles.valid = isWithinJointLimits(angles);
        return angles;
    }

    JointAngles computeTargetAngles(double x_rel, double y_rel, double z_rel) {
        double p_body_x = initial_foot_x_ + x_rel;
        double p_body_y = initial_foot_y_ + y_rel;
        double p_body_z = initial_foot_z_ + z_rel;

        double dx_shifted = p_body_x - x_off_;
        double dy_shifted = p_body_y - y_off_;
        
        double x_local = dx_shifted * std::cos(-theta_) - dy_shifted * std::sin(-theta_);
        double y_local = dx_shifted * std::sin(-theta_) + dy_shifted * std::cos(-theta_);
        double z_local = p_body_z;

        return solveIK(x_local, y_local, z_local);
    }
};

int main() {
    // Correct theta angles
    std::vector<LegDebug> legs = {
        LegDebug(1,  M_PI / 4.0),
        LegDebug(2, -M_PI / 4.0),
        LegDebug(3, -3.0 * M_PI / 4.0),
        LegDebug(4,  3.0 * M_PI / 4.0)
    };

    double vx = 0.04; // m/s
    double vy = 0.0;
    double GAIT_PERIOD = 3.0;
    double SWING_HEIGHT = 0.10;

    double Sx = vx * GAIT_PERIOD;
    double Sy = vy * GAIT_PERIOD;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Gait Simulation: Swing Phase analysis\n";
    std::cout << "Sx = " << Sx << ", Sy = " << Sy << ", SWING_HEIGHT = " << SWING_HEIGHT << "\n\n";

    for (int step = 0; step <= 10; ++step) {
        double sigma = step / 10.0; // 0 to 1
        double x_rel = -Sx / 2.0 + Sx * sigma;
        double y_rel = -Sy / 2.0 + Sy * sigma;
        double z_rel = SWING_HEIGHT * std::sin(M_PI * sigma);

        std::cout << "Sigma = " << sigma << " | rel: (" << x_rel << ", " << y_rel << ", " << z_rel << ")\n";
        for (auto& leg : legs) {
            JointAngles a = leg.computeTargetAngles(x_rel, y_rel, z_rel);
            std::cout << "  Leg " << leg.leg_id_ << ": " 
                      << (a.valid ? "VALID" : "INVALID") 
                      << " | q1 = " << (a.q1 * 180.0 / M_PI)
                      << ", q2 = " << (a.q2 * 180.0 / M_PI)
                      << ", q3 = " << (a.q3 * 180.0 / M_PI) << "\n";
        }
        std::cout << "\n";
    }

    return 0;
}
