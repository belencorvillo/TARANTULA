#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct JointAngles {
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
};

double ClosestPtSegmentSegment(
    const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q1, const Eigen::Vector3d& q2)
{
    Eigen::Vector3d d1 = p2 - p1;
    Eigen::Vector3d d2 = q2 - q1;
    Eigen::Vector3d r = p1 - q1;
    double a = d1.dot(d1);
    double e = d2.dot(d2);
    double f = d2.dot(r);

    double s, t;
    Eigen::Vector3d c1, c2;

    if (a <= 1e-8 && e <= 1e-8) {
        s = 0.0;
        t = 0.0;
        c1 = p1;
        c2 = q1;
        return (c1 - c2).squaredNorm();
    }
    if (a <= 1e-8) {
        s = 0.0;
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        double c = d1.dot(r);
        if (e <= 1e-8) {
            t = 0.0;
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            double b = d1.dot(d2);
            double denom = a * e - b * b;

            if (denom != 0.0) {
                s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
            } else {
                s = 0.0;
            }

            t = (b * s + f) / e;

            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }

    c1 = p1 + s * d1;
    c2 = q1 + t * d2;
    return (c1 - c2).squaredNorm();
}

std::vector<Eigen::Vector3d> getJointPositionsInBodyFrame(int leg_id, double theta, const JointAngles& a) {
    double L1 = 0.08;
    double L2 = 0.188;
    double L3 = 0.211;
    double R_HIP = 0.15;

    double x_off = R_HIP * std::cos(theta);
    double y_off = R_HIP * std::sin(theta);

    Eigen::Isometry3d leg_to_body = Eigen::Translation3d(x_off, y_off, 0.0) * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d j0_local(0.0, 0.0, 0.0);
    Eigen::Vector3d j1_local(L1 * std::cos(a.q1), L1 * std::sin(a.q1), 0.0);
    double r2 = L1 + L2 * std::cos(a.q2);
    Eigen::Vector3d j2_local(r2 * std::cos(a.q1), r2 * std::sin(a.q1), L2 * std::sin(a.q2));
    double r3 = L1 + L2 * std::cos(a.q2) + L3 * std::cos(a.q2 + a.q3);
    Eigen::Vector3d j3_local(r3 * std::cos(a.q1), r3 * std::sin(a.q1), L2 * std::sin(a.q2) + L3 * std::sin(a.q2 + a.q3));

    std::vector<Eigen::Vector3d> J(4);
    J[0] = leg_to_body * j0_local;
    J[1] = leg_to_body * j1_local;
    J[2] = leg_to_body * j2_local;
    J[3] = leg_to_body * j3_local;
    return J;
}

int main() {
    std::vector<double> thetas = {
         M_PI / 4.0,
        -M_PI / 4.0,
        -3.0 * M_PI / 4.0,
         3.0 * M_PI / 4.0
    };

    JointAngles nominal = { 0.0, 20.0 * M_PI / 180.0, -100.0 * M_PI / 180.0 };

    // Swing joint angles for Leg 1 (obtained from previous run)
    std::vector<JointAngles> leg1_swing = {
        { 0.029 * M_PI, 20.613 * M_PI / 180.0, -104.203 * M_PI / 180.0 }, // Sigma = 0
        { 0.023 * M_PI, 30.220 * M_PI / 180.0, -109.357 * M_PI / 180.0 }, // Sigma = 0.1
        { 0.017 * M_PI, 38.873 * M_PI / 180.0, -112.755 * M_PI / 180.0 }, // Sigma = 0.2
        { 0.011 * M_PI, 45.556 * M_PI / 180.0, -114.426 * M_PI / 180.0 }, // Sigma = 0.3
        { 0.005 * M_PI, 49.575 * M_PI / 180.0, -114.753 * M_PI / 180.0 }, // Sigma = 0.4
        { 0.000 * M_PI, 50.644 * M_PI / 180.0, -114.163 * M_PI / 180.0 }, // Sigma = 0.5 (Peak)
    };

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Checking distances during Leg 1 Swing phase:\n";

    for (size_t s = 0; s < leg1_swing.size(); ++s) {
        double sigma = s * 0.1;
        std::vector<std::vector<Eigen::Vector3d>> joints(4);
        joints[0] = getJointPositionsInBodyFrame(1, thetas[0], leg1_swing[s]);
        joints[1] = getJointPositionsInBodyFrame(2, thetas[1], nominal);
        joints[2] = getJointPositionsInBodyFrame(3, thetas[2], nominal);
        joints[3] = getJointPositionsInBodyFrame(4, thetas[3], nominal);

        double min_dist = 999.0;
        int min_other_leg = -1;

        for (int l = 1; l < 4; ++l) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    double dist_sq = ClosestPtSegmentSegment(
                        joints[0][i], joints[0][i+1],
                        joints[l][j], joints[l][j+1]
                    );
                    double dist = std::sqrt(dist_sq);
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_other_leg = l + 1;
                    }
                }
            }
        }
        std::cout << "Sigma = " << sigma << " | min distance to any other leg: " << min_dist << " m (Leg " << min_other_leg << ")\n";
    }

    return 0;
}
