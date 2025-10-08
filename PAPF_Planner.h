#pragma once

#include <vector>
#include "Vector2D.h"

struct Obstacle {
    Vector2D center;
    double radius;
};

struct USV {
    Vector2D position;
    double velocity = 0.0;
    double yaw = 0.0; //  in radians
};

class PAPF_Planner {
public:
    struct Params {
        double k_att = 10;
        double d_g = 4.0;

        double k_rep = 0.5;
        double d_o = 5.0;
        int n = 2;

        double k_prd = 1.0;
        double d_prd = 10.0;

        double max_turning_angle_deg = 20.0;

        double v_c = 1.0;
        double theta1_deg = 20.0;
        double theta2_deg = 45.0;
        double dt = 0.1;
    };

    PAPF_Planner(Params params);
    void computeStep(USV& usv, const Vector2D& goal, const std::vector<Obstacle>& obstacles);

private:
    Vector2D calculateAttractiveForce(const USV& usv, const Vector2D& goal);
    Vector2D calculateRepulsiveForce(const USV& usv, const std::vector<Obstacle>& obstacles, const Vector2D& goal);
    Vector2D calculatePredictiveForce(const USV& usv, const std::vector<Obstacle>& obstacles);
    
    double normalizeAngle(double angle);

    Params m_params;
};